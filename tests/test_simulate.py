"""Verification of the time-domain (backward) simulation.

The most important test here is the energy balance: every joule taken from
the cells must appear in exactly one sink. If someone changes the regen or
battery logic and forgets a term, this test fails.
"""

import numpy as np
import pytest

from evdynasim.cycles import DriveCycle, constant_speed, nedc
from evdynasim.params import with_changes
from evdynasim.simulate import ENERGY_SINKS, simulate_cycle, steady_state_power_w


def hill_cycle() -> DriveCycle:
    """Accelerate, climb, descend, stop. Exercises every branch of the model."""
    t = np.arange(0.0, 301.0)
    v = np.interp(t, [0, 30, 270, 300], [0, 25, 25, 0])
    g = np.interp(t, [0, 60, 120, 180, 240, 300], [0, 6, 6, -6, -6, 0])
    return DriveCycle("hill", t, v, g)


@pytest.mark.parametrize("cfg", ["simple", "ev"])
@pytest.mark.parametrize("cycle_factory", [nedc, hill_cycle])
def test_energy_balance_closes(cfg, cycle_factory, request):
    p = request.getfixturevalue(cfg)
    s = simulate_cycle(p, cycle_factory()).summary
    assert sum(s[k] for k in ENERGY_SINKS) == pytest.approx(s["chemical_energy_kwh"], rel=1e-10)


def test_energy_balance_closes_with_regen_curtailment(ev):
    full = with_changes(ev, {"battery.initial_soc": 0.995})  # charge limit 4 kW
    s = simulate_cycle(full, hill_cycle()).summary
    assert s["regen_curtailed_kwh"] > 0.0
    assert sum(s[k] for k in ENERGY_SINKS) == pytest.approx(s["chemical_energy_kwh"], rel=1e-10)


def test_kinetic_energy_returns_to_zero_on_stop_to_stop_cycle(ev):
    assert simulate_cycle(ev, nedc()).summary["kinetic_change_kwh"] == pytest.approx(0, abs=1e-12)


def test_simple_model_reproduces_v02_results(simple):
    # regression values of the v0.2 model (docs/03, section 3.8)
    s = simulate_cycle(simple, nedc()).summary
    assert s["distance_km"] == pytest.approx(11.0132, abs=1e-4)
    assert s["battery_energy_kwh"] == pytest.approx(1.4558, abs=1e-4)


def test_constant_speed_matches_steady_state_model(simple):
    res = simulate_cycle(simple, constant_speed(100.0, 600.0))
    np.testing.assert_allclose(res.battery_power_w, steady_state_power_w(simple, 100.0))


def test_constant_speed_consumption(simple):
    s = simulate_cycle(simple, constant_speed(100.0, 3600.0)).summary
    assert s["distance_km"] == pytest.approx(100.0)
    assert s["consumption_kwh_per_100km"] == pytest.approx(15.344, abs=0.002)


def test_soc_change_matches_charge_throughput(ev):
    res = simulate_cycle(ev, nedc())
    charge_ah = np.sum(res.current_a * res.dt_s) / 3600.0
    assert res.soc[0] - res.soc[-1] == pytest.approx(charge_ah / ev.battery.capacity_ah)


def test_detailed_model_uses_more_energy_than_simple(simple, ev):
    e_simple = simulate_cycle(simple, nedc()).summary["consumption_kwh_per_100km"]
    e_ev = simulate_cycle(ev, nedc()).summary["consumption_kwh_per_100km"]
    assert 12.0 < e_simple < e_ev < 18.0


def test_regen_reduces_consumption(ev):
    with_regen = simulate_cycle(ev, nedc()).summary["battery_energy_kwh"]
    no_regen = simulate_cycle(with_changes(ev, {"brakes.regen_fraction": 0.0}), nedc()).summary
    assert no_regen["regen_energy_kwh"] == 0.0
    assert with_regen < no_regen["battery_energy_kwh"]


def test_regen_power_limit_respected(ev):
    limited = with_changes(ev, {"brakes.max_regen_power_w": 5_000.0})
    res = simulate_cycle(limited, hill_cycle())
    assert res.regen_wheel_w.max() <= 5_000.0 + 1e-9


def test_no_regen_below_min_speed(ev):
    res = simulate_cycle(ev, nedc())
    slow = res.speed_mps < ev.brakes.min_regen_speed_mps
    assert np.all(res.regen_wheel_w[slow] == 0.0)


def test_cold_weather_increases_consumption(ev):
    warm = simulate_cycle(ev, nedc()).summary["consumption_kwh_per_100km"]
    cold_p = with_changes(ev, {"env.ambient_temp_c": -10.0, "thermal.initial_temp_c": -10.0})
    cold = simulate_cycle(cold_p, nedc()).summary
    assert cold["consumption_kwh_per_100km"] > warm * 1.2  # HVAC heating dominates
    assert cold["battery_loss_kwh"] > 2 * simulate_cycle(ev, nedc()).summary["battery_loss_kwh"]


def test_weak_motor_is_flagged(ev):
    weak = with_changes(ev, {"motor.max_power_w": 20_000.0})
    assert simulate_cycle(weak, nedc()).summary["torque_limited_steps"] > 0


def test_dataframe_export(ev):
    df = simulate_cycle(ev, nedc()).to_dataframe()
    assert len(df) == 1180
    assert {"t_s", "speed_kmh", "p_battery_kw", "soc_end", "temp_end_c"} <= set(df.columns)
