"""Verification of the drive-cycle input layer."""

import numpy as np
import pytest

from evdynasim.cycles import DriveCycle, constant_speed, get_cycle, load_csv, nedc


def test_nedc_duration_and_max_speed():
    c = nedc()
    assert c.duration_s == 1180.0
    assert c.speed_mps.max() * 3.6 == pytest.approx(120.0)


def test_nedc_distance_matches_published_value():
    # UNECE R83: 4 x 1.013 km + 6.955 km = 11.007 km. Our gear-change
    # interpretation gives a slightly different ECE distance, so allow 0.2 %.
    assert nedc().distance_m / 1000 == pytest.approx(11.007, rel=0.002)


def test_nedc_starts_and_ends_at_standstill():
    c = nedc()
    assert c.speed_mps[0] == 0.0 and c.speed_mps[-1] == 0.0


def test_constant_speed_distance():
    c = constant_speed(72.0, 100.0)  # 20 m/s for 100 s
    assert c.distance_m == pytest.approx(2000.0)


def test_csv_roundtrip(tmp_path):
    f = tmp_path / "drive.csv"
    f.write_text("time_s,speed_kmh,grade_pct\n0,0,0\n1,3.6,2\n2,7.2,2\n")
    c = load_csv(f)
    assert c.name == "drive"
    np.testing.assert_allclose(c.speed_mps, [0.0, 1.0, 2.0])
    np.testing.assert_allclose(c.grade_pct, [0.0, 2.0, 2.0])


def test_csv_without_grade_defaults_to_flat(tmp_path):
    f = tmp_path / "flat.csv"
    f.write_text("time_s,speed_kmh\n0,0\n1,10\n")
    assert np.all(load_csv(f).grade_pct == 0.0)


def test_csv_missing_column_rejected(tmp_path):
    f = tmp_path / "bad.csv"
    f.write_text("time,speed\n0,0\n1,1\n")
    with pytest.raises(ValueError, match="missing column"):
        load_csv(f)


def test_get_cycle_by_name():
    assert get_cycle("NEDC").name == "NEDC"


@pytest.mark.parametrize(
    "t, v",
    [([0, 1, 1], [0, 1, 2]),     # time not increasing
     ([0, 1, 2], [0, -1, 0]),    # negative speed
     ([0, 1, 2], [0, np.nan, 0]),  # NaN
     ([0], [0])],                # too short
)  # fmt: skip
def test_invalid_cycles_rejected(t, v):
    with pytest.raises(ValueError):
        DriveCycle("bad", np.array(t, float), np.array(v, float), np.zeros(len(t)))
