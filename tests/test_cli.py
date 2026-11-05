"""End-to-end tests of the command line tool and the studies."""

import json

import pytest

from evdynasim import cpp_engine, studies
from evdynasim.cli import main
from evdynasim.cycles import nedc

from .conftest import CONFIG_DIR

CONFIG = str(CONFIG_DIR / "generic_ev.yaml")


def test_steady_command(capsys):
    assert main(["steady", "--config", CONFIG, "--speed", "100"]) == 0
    assert "16.37 kW" in capsys.readouterr().out


def test_cycle_command_writes_outputs(tmp_path):
    assert main(["cycle", "--config", CONFIG, "--cycle", "nedc", "--out", str(tmp_path)]) == 0
    for name in ["nedc_timeseries.csv", "nedc_summary.json", "nedc_cycle.png", "nedc_energy.png",
                 "nedc_motor_map.png"]:  # fmt: skip
        assert (tmp_path / name).exists(), name
    summary = json.loads((tmp_path / "nedc_summary.json").read_text())
    assert summary["distance_km"] > 10.0


@pytest.mark.skipif(not cpp_engine.is_available(), reason="C++ engine not built")
def test_cycle_command_cpp_engine(capsys):
    assert main(["cycle", "--config", CONFIG, "--engine", "cpp"]) == 0
    assert "consumption_kwh_per_100km" in capsys.readouterr().out


def test_validate_command_on_csv_files(tmp_path, ev):
    from evdynasim.synthetic import synthesize_measurement

    files = []
    for seed, temp in [(21, 0.0), (22, 20.0), (23, 10.0)]:
        f = tmp_path / f"d{seed}.csv"
        synthesize_measurement(ev, seed, temp, structural=False, duration_s=400).to_csv(f)
        files.append(str(f))
    out = tmp_path / "val"
    assert main(["validate", "--config", CONFIG, "--drives", *files, "--n-train", "2",
                 "--out", str(out)]) == 0  # fmt: skip
    assert (out / "report.md").exists()
    assert (out / "calibrated_vehicle.yaml").exists()


def test_sensitivity_ranks_drag_first_on_motorway(ev):
    df = studies.sensitivity(ev, studies.motorway_cycle(120.0, 900.0))
    assert df.iloc[0]["key"] == "body.drag_coefficient"
    city = studies.sensitivity(ev, nedc())
    assert city.iloc[0]["key"] == "body.mass_kg"


def test_ambient_sweep_minimum_near_setpoint(ev):
    df = studies.ambient_sweep(ev, nedc(), temps_c=(-10, 20, 35))
    cons = df.set_index("ambient_c")["consumption_kwh_per_100km"]
    assert cons[20] < cons[35] < cons[-10]
