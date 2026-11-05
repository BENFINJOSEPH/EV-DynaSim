"""Longitudinal road-load forces (pure functions, SI units, NumPy-vectorised).

Every function accepts a scalar or a NumPy array and returns the same shape.
Sign convention: positive force opposes forward motion (it is a load the
powertrain has to overcome). The grade force is negative downhill.

Assumptions
-----------
* vehicle drives forward only (speed >= 0)
* no wind (air speed = ground speed)
* constant rolling resistance coefficient, no speed dependence
* road grade given in percent (rise/run * 100)
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from evdynasim.params import ModelParams


def _as_speed(speed_mps: ArrayLike) -> NDArray[np.float64]:
    v = np.asarray(speed_mps, dtype=float)
    if np.any(v < 0):
        raise ValueError("Speed must not be negative (forward driving only).")
    return v


def grade_angle_rad(grade_pct: ArrayLike) -> NDArray[np.float64]:
    """Convert road grade in percent to the road angle theta in radians."""
    return np.arctan(np.asarray(grade_pct, dtype=float) / 100.0)


def aero_drag_n(p: ModelParams, speed_mps: ArrayLike) -> NDArray[np.float64]:
    """F_aero = 1/2 * rho * Cd * A * v^2."""
    v = _as_speed(speed_mps)
    b = p.body
    return 0.5 * p.env.air_density_kg_m3 * b.drag_coefficient * b.frontal_area_m2 * v**2


def rolling_resistance_n(
    p: ModelParams, speed_mps: ArrayLike, grade_pct: ArrayLike = 0.0
) -> NDArray[np.float64]:
    """F_roll = m * g * Crr * cos(theta), zero when the vehicle stands still."""
    v = _as_speed(speed_mps)
    theta = grade_angle_rad(grade_pct)
    force = p.body.mass_kg * p.env.gravity_m_s2 * p.body.rolling_coefficient * np.cos(theta)
    return np.where(v > 0.0, force, 0.0)


def grade_force_n(p: ModelParams, grade_pct: ArrayLike) -> NDArray[np.float64]:
    """F_grade = m * g * sin(theta). Positive uphill, negative downhill."""
    return p.body.mass_kg * p.env.gravity_m_s2 * np.sin(grade_angle_rad(grade_pct))


def inertial_force_n(p: ModelParams, accel_mps2: ArrayLike) -> NDArray[np.float64]:
    """F_acc = m_eff * a, with m_eff including rotating inertia."""
    return p.body.effective_mass_kg * np.asarray(accel_mps2, dtype=float)


def resistance_force_n(
    p: ModelParams, speed_mps: ArrayLike, grade_pct: ArrayLike = 0.0
) -> NDArray[np.float64]:
    """All forces except inertia: F_aero + F_roll + F_grade."""
    return (
        aero_drag_n(p, speed_mps)
        + rolling_resistance_n(p, speed_mps, grade_pct)
        + grade_force_n(p, grade_pct)
    )


def traction_force_n(
    p: ModelParams,
    speed_mps: ArrayLike,
    accel_mps2: ArrayLike = 0.0,
    grade_pct: ArrayLike = 0.0,
) -> NDArray[np.float64]:
    """Total force required at the wheels: F = m_eff*a + F_aero + F_roll + F_grade."""
    return inertial_force_n(p, accel_mps2) + resistance_force_n(p, speed_mps, grade_pct)
