"""Forward simulation: integrator accuracy, tracking, limits, anti-windup."""

import numpy as np
import pytest

from evdynasim.cycles import nedc
from evdynasim.forward import (
    DriverConfig,
    coastdown_analytical,
    max_drive_force_n,
    simulate_coastdown,
    simulate_forward,
)
from evdynasim.params import with_changes
from evdynasim.simulate import simulate_cycle


@pytest.mark.parametrize("method, order", [("euler", 1), ("rk4", 4)])
def test_integrator_order_on_analytical_coastdown(ev, method, order):
    exact = coastdown_analytical(ev, 30.0, [60.0])[0]
    errors = [abs(simulate_coastdown(ev, 30.0, 60.0, h, method) - exact) for h in (2.0, 1.0, 0.5)]
    observed = np.log2(errors[1] / errors[2])
    assert observed == pytest.approx(order, abs=0.15)


def test_coastdown_reaches_standstill(ev):
    assert coastdown_analytical(ev, 10.0, [10_000.0])[0] == 0.0


def test_drive_force_envelope(ev):
    low = max_drive_force_n(ev, 5.0)  # constant torque region
    assert low == pytest.approx(310 * 9 * 0.97 / 0.33)
    assert max_drive_force_n(ev, 40.0) < low  # constant power region
    assert max_drive_force_n(ev, 60.0) == 0.0  # above max motor speed


@pytest.fixture(scope="module")
def nedc_forward():
    from evdynasim.params import load_params

    from .conftest import CONFIG_DIR

    p = load_params(CONFIG_DIR / "generic_ev.yaml")
    return p, simulate_forward(p, nedc())


def test_forward_tracks_cycle(nedc_forward):
    _, r = nedc_forward
    assert r.summary["speed_rmse_kmh"] < 0.1
    assert r.summary["saturated_fraction"] == 0.0
    assert np.all(r.speed_mps >= 0.0)


def test_forward_energy_agrees_with_backward(nedc_forward):
    p, r = nedc_forward
    backward = simulate_cycle(p, nedc()).summary["battery_energy_kwh"]
    assert r.summary["battery_energy_kwh"] == pytest.approx(backward, rel=0.01)


def test_anti_windup_prevents_overshoot(ev):
    weak = with_changes(ev, {"motor.max_power_w": 25_000.0})
    with_aw = simulate_forward(weak, nedc(), DriverConfig(anti_windup=True)).summary
    without = simulate_forward(weak, nedc(), DriverConfig(anti_windup=False)).summary
    assert with_aw["saturated_fraction"] > 0.01  # the motor really is too weak
    assert with_aw["max_overshoot_kmh"] < 1.0
    assert without["max_overshoot_kmh"] > 10.0
