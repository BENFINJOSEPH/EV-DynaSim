"""Charging system tests, written as a requirements-based test specification.

Every test names the requirement it verifies (REQ-CHG-xx). The full table
with rationale is in docs/09_charging.md. The safety requirements (voltage,
current, power, temperature) are checked over a scenario matrix of ambient
temperature x charger power, including a cold battery and a hot day.
"""

import itertools

import numpy as np
import pytest

from evdynasim.charging import (
    ambient_study,
    charger_power_study,
    simulate_charging,
)
from evdynasim.params import load_params, with_changes

from .conftest import CONFIG_DIR

AMBIENTS = (-10.0, 25.0, 40.0)
CHARGERS = (50.0, 350.0)
SCENARIOS = list(itertools.product(AMBIENTS, CHARGERS))


@pytest.fixture(scope="module")
def p():
    return load_params(CONFIG_DIR / "generic_ev.yaml")


@pytest.fixture(scope="module")
def matrix(p):
    """Full charge 10 -> 100 % for every scenario (computed once, shared by the tests)."""
    return {
        (amb, kw): simulate_charging(p, 0.10, 1.00, ambient_temp_c=amb, charger_power_kw=kw)
        for amb, kw in SCENARIOS
    }


def ids(scenario):
    return f"{scenario[0]:+.0f}degC-{scenario[1]:.0f}kW"


# ---- safety limits over the scenario matrix -------------------------------------------
@pytest.mark.parametrize("scenario", SCENARIOS, ids=ids)
def test_req_chg_01_terminal_voltage_never_exceeds_cv_limit(p, matrix, scenario):
    r = matrix[scenario]
    v_max = p.charging.cv_cell_voltage_v * p.battery.cells_series
    assert r.series["terminal_v"].max() <= v_max + 1e-9


@pytest.mark.parametrize("scenario", SCENARIOS, ids=ids)
def test_req_chg_02_current_never_exceeds_derated_c_rate(p, matrix, scenario):
    s = matrix[scenario].series
    i_max = p.charging.max_c_rate * p.battery.capacity_ah
    assert s["charge_current_a"].max() <= i_max + 1e-9
    # and never above the derated limit the BMS computed for that step
    assert np.all(s["charge_current_a"] <= s["limit_cc_a"] + 1e-9)


@pytest.mark.parametrize("scenario", SCENARIOS, ids=ids)
def test_req_chg_03_power_never_exceeds_charger_rating(matrix, scenario):
    s = matrix[scenario].series
    assert s["power_in_kw"].max() <= scenario[1] + 1e-9


@pytest.mark.parametrize("scenario", SCENARIOS, ids=ids)
def test_req_chg_06_soc_increases_monotonically_and_stays_below_one(matrix, scenario):
    s = matrix[scenario].series
    assert np.all(np.diff(s["soc_end"]) > 0)
    assert s["soc_end"].max() <= 1.0


@pytest.mark.parametrize("scenario", SCENARIOS, ids=ids)
def test_req_chg_07_energy_balance_closes(matrix, scenario):
    r = matrix[scenario].summary
    total = r["charger_loss_kwh"] + r["battery_loss_kwh"] + r["energy_stored_kwh"]
    assert total == pytest.approx(r["energy_grid_kwh"], rel=1e-10)


@pytest.mark.parametrize("scenario", SCENARIOS, ids=ids)
def test_req_chg_08_current_never_rises_in_cv_phase(matrix, scenario):
    s = matrix[scenario].series
    cv = s[s["phase"] == "CV"]["charge_current_a"].to_numpy()
    assert np.all(np.diff(cv) <= 1e-9)


# ---- temperature protection --------------------------------------------------------------
def test_req_chg_04_no_charging_at_or_above_max_temperature(p):
    # worst case: hot day, battery already hot, no active cooling, strong charger
    hot = with_changes(p, {"charging.cooling_w_k": 0.0})
    r = simulate_charging(hot, 0.1, 1.0, ambient_temp_c=40.0, initial_temp_c=48.0,
                          charger_power_kw=350.0)  # fmt: skip
    s = r.series
    start_temp = np.concatenate([[48.0], s["temp_end_c"].to_numpy()[:-1]])
    assert np.all(start_temp[s["charge_current_a"] > 0] < p.charging.max_cell_temp_c)
    assert r.summary["max_temp_c"] < p.charging.max_cell_temp_c + 0.5


def test_req_chg_04_charging_stops_when_battery_is_too_hot(p):
    r = simulate_charging(p, 0.1, 0.8, ambient_temp_c=40.0, initial_temp_c=56.0)
    assert r.summary["stop_reason"] == "battery temperature limit"
    assert r.series.empty


def test_req_chg_04_hot_battery_is_derated(p):
    s = simulate_charging(p, 0.1, 0.8, ambient_temp_c=40.0, initial_temp_c=48.0).series
    assert s["charge_current_a"].iloc[0] < p.charging.max_c_rate * p.battery.capacity_ah


# ---- stop conditions -----------------------------------------------------------------------
def test_req_chg_05_stops_at_target_soc(p):
    r = simulate_charging(p, 0.10, 0.80)
    assert r.summary["stop_reason"] == "target SOC reached"
    s = r.series
    assert (
        s["soc_start"].iloc[-1] < 0.80 <= s["soc_end"].iloc[-1]
    )  # stops in the first step past target


def test_req_chg_12_full_charge_ends_with_cv_taper_at_c20(p):
    r = simulate_charging(p, 0.10, 1.00)
    assert r.summary["stop_reason"] == "end of CV phase (current below C/20)"
    assert r.summary["time_CV_min"] > 0.0
    assert r.series["charge_current_a"].iloc[-1] >= 0.05 * p.battery.capacity_ah


# ---- performance requirements ----------------------------------------------------------------
def test_req_chg_09_cold_battery_charges_slower_and_preconditioning_helps(p):
    df = ambient_study(p, temps_c=(0, 25))
    t = df.set_index(["ambient_c", "battery_start"])["time_min"]
    assert t[(0, "parked")] > 2 * t[(25, "parked")]
    assert t[(0, "preconditioned")] == pytest.approx(t[(25, "parked")], rel=0.05)


def test_req_chg_10_more_charger_power_never_slows_charging(p):
    df = charger_power_study(p, powers_kw=(50, 100, 150, 350))
    assert np.all(np.diff(df["time_min"]) <= 1e-9)
    # above the battery's own limit, extra station power gives nothing
    assert df["time_min"].iloc[-1] == pytest.approx(df["time_min"].iloc[-2])


def test_req_chg_13_typical_fast_charge_time(p):
    # design target for the generic vehicle: 10 -> 80 % in under 30 min at 25 degC on 150 kW
    assert simulate_charging(p, 0.10, 0.80).summary["duration_min"] < 30.0


# ---- input validation ----------------------------------------------------------------------
def test_req_chg_11_invalid_requests_rejected(p, simple):
    with pytest.raises(ValueError):
        simulate_charging(p, 0.8, 0.5)
    with pytest.raises(ValueError, match="no 'charging' section"):
        simulate_charging(simple)


def test_charging_config_validation(p):
    from evdynasim.interp import Table1D

    with pytest.raises(ValueError):
        with_changes(p, {"charging.derate_soc": Table1D([0.0, 1.0], [1.0, 1.5])})
