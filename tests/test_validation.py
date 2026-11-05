"""Tests for measured-data import, calibration and validation."""

import numpy as np
import pandas as pd
import pytest

from evdynasim.calibration import calibrate, evaluate
from evdynasim.measured import interval_to_sample_power, load_mapping, load_measured_csv
from evdynasim.report import write_calibrated_config
from evdynasim.synthetic import TRUTH_PARAMS, random_drive, synthesize_measurement

from .conftest import CONFIG_DIR


# ---- import --------------------------------------------------------------------
def test_loader_with_voltage_current_and_cleaning(tmp_path):
    f = tmp_path / "log.csv"
    pd.DataFrame(
        {
            "t_ms": [2000, 0, 1000, 1000, 3000, 4000],  # unsorted + duplicate
            "v_kmh": [36.0, 0.0, 18.0, 18.0, np.nan, 36.0],  # missing value
            "u": [400.0] * 6,
            "i": [-50.0, 0.0, -25.0, -25.0, -50.0, -50.0],  # logger: discharge negative
            "temp": [12.0] * 6,
        }
    ).to_csv(f, index=False)
    mapping = {
        "time": {"column": "t_ms", "scale": 0.001},
        "speed_mps": {"column": "v_kmh", "scale": 1 / 3.6},
        "battery_voltage_v": {"column": "u"},
        "battery_current_a": {"column": "i"},
        "current_sign": -1.0,
        "ambient_temp_c": {"column": "temp"},
    }
    d = load_measured_csv(f, mapping)
    np.testing.assert_allclose(d.time_s, [0, 1, 2, 3, 4])
    np.testing.assert_allclose(d.speed_mps, [0, 5, 10, 10, 10])  # NaN row dropped, then resampled
    assert d.battery_power_w[2] == pytest.approx(20_000.0)
    assert d.ambient_temp_c == 12.0
    assert np.all(d.grade_pct == 0.0)


def test_mapping_files_are_valid():
    m = load_mapping(CONFIG_DIR / "datasets" / "csv_mapping_example.yaml")
    assert m["speed_mps"]["scale"] == pytest.approx(1 / 3.6)
    t = load_mapping(CONFIG_DIR / "datasets" / "tum_taycan_template.yaml")
    assert "battery_current_a" in t


def test_csv_roundtrip_of_measured_drive(ev, tmp_path):
    d = synthesize_measurement(ev, 3, 15.0, duration_s=300)
    d.to_csv(tmp_path / "d.csv")
    back = load_measured_csv(tmp_path / "d.csv")
    np.testing.assert_allclose(back.speed_mps, d.speed_mps, rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(back.battery_power_w, d.battery_power_w, rtol=1e-5, atol=1e-2)


def test_interval_to_sample_power():
    np.testing.assert_allclose(interval_to_sample_power(np.array([2.0, 4.0])), [2.0, 3.0, 4.0])


def test_random_drive_is_physically_plausible():
    c = random_drive(5)
    accel = np.diff(c.speed_mps)
    assert np.abs(accel).max() <= 2.5 + 1e-9  # m/s^2 at dt = 1 s
    assert c.speed_mps[0] == 0 and c.speed_mps[-1] == 0
    assert c.speed_mps.max() * 3.6 <= 135


# ---- calibration -----------------------------------------------------------------
def test_perfect_model_has_only_noise_error(ev):
    from evdynasim.synthetic import truth_vehicle

    d = synthesize_measurement(ev, 7, 20.0, power_noise_w=0, speed_noise_mps=0, structural=False,
                               duration_s=400)  # fmt: skip
    m = evaluate(truth_vehicle(ev, structural=False), [d]).iloc[0]
    assert m.rmse_kw == pytest.approx(0.0, abs=1e-6)
    assert m.energy_error_pct == pytest.approx(0.0, abs=0.05)


def test_calibration_recovers_true_parameters(ev):
    """With matched model structure the estimates must be close to the truth."""
    drives = [
        synthesize_measurement(ev, s, temp, structural=False, duration_s=700)
        for s, temp in [(11, -5.0), (12, 25.0), (13, 10.0)]
    ]
    cal = calibrate(ev, drives)
    est = dict(zip(cal.table.parameter, cal.table.estimate, strict=True))
    for key, true in TRUTH_PARAMS.items():
        assert est[key] == pytest.approx(true, rel=0.06), key
    assert cal.rmse_after_kw < cal.rmse_before_kw


def test_write_calibrated_config(ev, tmp_path):
    from evdynasim.params import load_params, with_changes

    q = with_changes(ev, {"body.rolling_coefficient": 0.0123, "aux.base_power_w": 700.0})
    out = write_calibrated_config(
        CONFIG_DIR / "generic_ev.yaml", tmp_path / "cal.yaml", q,
        ["body.rolling_coefficient", "aux.base_power_w"],
    )  # fmt: skip
    back = load_params(out)
    assert back.body.rolling_coefficient == pytest.approx(0.0123)
    assert back.aux.base_power_w == pytest.approx(700.0)
    assert back.motor.loss_map is not None  # relative path fixed
