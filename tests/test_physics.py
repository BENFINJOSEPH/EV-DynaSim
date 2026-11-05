"""Verification of the road-load force functions.

Three kinds of test:
1. hand-calculated reference values (numbers documented in docs/03_physics_model.md)
2. physical properties that must always hold (scaling, symmetry)
3. invalid input is rejected
"""

import numpy as np
import pytest

from evdynasim import physics
from evdynasim.params import with_changes
from evdynasim.simulate import steady_state_power_w

V100 = 100 / 3.6  # 100 km/h in m/s


# 1. reference values -------------------------------------------------------
def test_aero_drag_at_100_kmh(simple):
    # 0.5 * 1.225 * 0.28 * 2.3 * (100/3.6)^2 = 304.36 N
    assert physics.aero_drag_n(simple, V100) == pytest.approx(304.36, abs=0.01)


def test_rolling_resistance_flat(simple):
    # 1800 * 9.81 * 0.010 = 176.58 N
    assert physics.rolling_resistance_n(simple, V100) == pytest.approx(176.58)


def test_steady_state_battery_power_at_100_kmh(simple):
    # (304.36 + 176.58) * 27.778 / 0.9 + 500 = 15344 W
    assert steady_state_power_w(simple, 100.0) == pytest.approx(15344.0, abs=2.0)


def test_grade_force_10_percent(simple):
    # theta = atan(0.1); F = 1800 * 9.81 * sin(theta) = 1757.0 N
    assert physics.grade_force_n(simple, 10.0) == pytest.approx(1757.0, abs=0.5)


# 2. physical properties ----------------------------------------------------
def test_aero_drag_zero_at_standstill(simple):
    assert physics.aero_drag_n(simple, 0.0) == 0.0


def test_aero_drag_quadruples_when_speed_doubles(simple):
    assert physics.aero_drag_n(simple, 2 * V100) == pytest.approx(
        4 * physics.aero_drag_n(simple, V100)
    )


def test_no_rolling_resistance_at_standstill(simple):
    assert physics.rolling_resistance_n(simple, 0.0) == 0.0


def test_grade_force_is_antisymmetric(simple):
    assert physics.grade_force_n(simple, -6.0) == pytest.approx(-physics.grade_force_n(simple, 6.0))


def test_traction_force_is_sum_of_components(simple):
    v, a, g = 15.0, 0.8, 3.0
    expected = (
        physics.aero_drag_n(simple, v)
        + physics.rolling_resistance_n(simple, v, g)
        + physics.grade_force_n(simple, g)
        + physics.inertial_force_n(simple, a)
    )
    assert physics.traction_force_n(simple, v, a, g) == pytest.approx(expected)


def test_functions_are_vectorised(simple):
    v = np.array([0.0, 10.0, 20.0, 30.0])
    assert physics.traction_force_n(simple, v).shape == v.shape


def test_rotating_mass_factor_increases_inertial_force(simple):
    heavy = with_changes(simple, {"body.rotating_mass_factor": 1.05})
    assert physics.inertial_force_n(heavy, 1.0) == pytest.approx(1.05 * simple.body.mass_kg)


def test_steady_power_monotonic_in_speed(simple):
    p = steady_state_power_w(simple, np.linspace(0, 160, 50))
    assert np.all(np.diff(p) > 0)


# 3. invalid input ------------------------------------------------------------
def test_negative_speed_rejected(simple):
    with pytest.raises(ValueError):
        physics.aero_drag_n(simple, -1.0)


@pytest.mark.parametrize(
    "field, value",
    [("body.mass_kg", 0.0), ("motor.constant_efficiency", 1.2), ("brakes.regen_fraction", -0.1),
     ("body.rotating_mass_factor", 0.9), ("battery.initial_soc", 1.5),
     ("drivetrain.gearbox_efficiency", 0.0), ("thermal.mass_kg", -1.0)],
)  # fmt: skip
def test_invalid_parameters_rejected(simple, field, value):
    with pytest.raises(ValueError):
        with_changes(simple, {field: value})
