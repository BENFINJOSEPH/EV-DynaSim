"""Forward-facing simulation: a driver controller tracks the cycle.

Backward model: speed in -> power out. It cannot tell whether the vehicle is
able to follow the cycle.
Forward model: force in -> speed out, by integrating the equation of motion

    m_eff * dv/dt = F_drive - F_aero(v) - F_roll(v) - F_grade(s)
    ds/dt = v

F_drive is computed by a driver model (PI speed controller with
feed-forward) and limited by the motor torque envelope and the brakes.

Controller
----------
    F_ff  = m_eff * a_ref + F_res(v_ref)             (inverse model of the vehicle)
    e     = v_ref - v
    F_cmd = F_ff + Kp * e + Ki * integral(e)
    F     = clip(F_cmd, F_min, F_max(v))

Anti-windup by conditional integration: the integrator is frozen while the
output is saturated and the error would push it further into saturation.
Without it, the integral grows during a long saturation (for example a weak
motor on a motorway acceleration) and causes a large overshoot afterwards.

Numerics
--------
Controller runs at a fixed sample time (default 0.1 s, zero-order hold).
The plant is integrated over each sample with explicit Euler or classical
Runge-Kutta 4. Orders of accuracy (1 and 4) are verified against the
analytical coast-down solution in tests/test_forward.py.

Energy
------
The achieved trajectory (speed and grade along the actually driven distance)
is passed to the backward model, so energy is computed by exactly the same
powertrain and battery code. One model for energy, two ways to obtain the
motion.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from evdynasim import physics
from evdynasim.cycles import DriveCycle
from evdynasim.params import ModelParams
from evdynasim.simulate import SimulationResult, simulate_cycle

State = NDArray[np.float64]


# ---- integrators -----------------------------------------------------------------
def euler_step(f: Callable[[float, State], State], t: float, x: State, h: float) -> State:
    return x + h * f(t, x)


def rk4_step(f: Callable[[float, State], State], t: float, x: State, h: float) -> State:
    k1 = f(t, x)
    k2 = f(t + h / 2, x + h / 2 * k1)
    k3 = f(t + h / 2, x + h / 2 * k2)
    k4 = f(t + h, x + h * k3)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


INTEGRATORS = {"euler": euler_step, "rk4": rk4_step}


def coastdown_analytical(p: ModelParams, v0: float, t: NDArray[np.float64]) -> NDArray[np.float64]:
    """Exact speed while coasting on a flat road: m dv/dt = -(A + B v^2).

    v(t) = sqrt(A/B) * tan( atan(v0 sqrt(B/A)) - sqrt(A B) t / m ), until v = 0.
    """
    m = p.body.effective_mass_kg
    a = p.body.mass_kg * p.env.gravity_m_s2 * p.body.rolling_coefficient
    b = 0.5 * p.env.air_density_kg_m3 * p.body.drag_coefficient * p.body.frontal_area_m2
    phase = np.arctan(v0 * np.sqrt(b / a)) - np.sqrt(a * b) * np.asarray(t) / m
    return np.where(phase > 0, np.sqrt(a / b) * np.tan(np.maximum(phase, 0.0)), 0.0)


def simulate_coastdown(p: ModelParams, v0: float, t_end: float, h: float, method: str):
    """Integrate the coast-down with a given method and step size."""
    step = INTEGRATORS[method]

    def f(_t, x):
        v = max(x[0], 0.0)
        return np.array([-physics.resistance_force_n(p, v) / p.body.effective_mass_kg])

    n = int(round(t_end / h))
    x = np.array([v0])
    for k in range(n):
        x = step(f, k * h, x, h)
    return float(x[0])


# ---- driver and vehicle -------------------------------------------------------------
@dataclass(frozen=True)
class DriverConfig:
    sample_time_s: float = 0.1
    bandwidth_rad_s: float = 2.0  # Kp = m_eff * bandwidth
    integral_time_s: float = 5.0  # Ki = Kp / Ti
    anti_windup: bool = True
    feedforward: bool = True
    integrator: str = "rk4"
    max_brake_decel_mps2: float = 9.0  # friction + regen capability


@dataclass
class ForwardResult:
    time_s: NDArray[np.float64]
    speed_ref_mps: NDArray[np.float64]
    speed_mps: NDArray[np.float64]
    distance_m: NDArray[np.float64]
    grade_pct: NDArray[np.float64]
    force_cmd_n: NDArray[np.float64]  # controller output before limits
    force_n: NDArray[np.float64]  # applied force after limits
    saturated: NDArray[np.bool_]
    energy: SimulationResult  # backward model on the achieved trajectory
    summary: dict[str, float]


def max_drive_force_n(p: ModelParams, v: float) -> float:
    """Traction force available from the motor torque envelope."""
    w = v / p.body.wheel_radius_m * p.drivetrain.gear_ratio
    if w > p.motor.max_speed_rads:
        return 0.0
    t_max = float(p.motor.max_torque_at(w))
    return t_max * p.drivetrain.gear_ratio * p.drivetrain.gearbox_efficiency / p.body.wheel_radius_m


def simulate_forward(
    p: ModelParams, cycle: DriveCycle, cfg: DriverConfig | None = None
) -> ForwardResult:
    cfg = cfg or DriverConfig()
    h = cfg.sample_time_s
    m = p.body.effective_mass_kg
    kp = m * cfg.bandwidth_rad_s
    ki = kp / cfg.integral_time_s
    step = INTEGRATORS[cfg.integrator]

    # reference: speed, acceleration (interval based) and grade along the reference distance
    t_cyc, v_cyc = cycle.time_s, cycle.speed_mps
    a_cyc = np.diff(v_cyc) / np.diff(t_cyc)
    s_ref = np.concatenate([[0.0], np.cumsum(0.5 * (v_cyc[:-1] + v_cyc[1:]) * np.diff(t_cyc))])

    def grade_at(s: float) -> float:
        return float(np.interp(s, s_ref, cycle.grade_pct))

    t = np.arange(t_cyc[0], t_cyc[-1] + 0.5 * h, h)
    n = len(t)
    v_ref = np.interp(t, t_cyc, v_cyc)
    idx = np.clip(np.searchsorted(t_cyc, t, side="right") - 1, 0, len(a_cyc) - 1)
    a_ref = a_cyc[idx]

    v_out, s_out, g_out = np.zeros(n), np.zeros(n), np.zeros(n)
    f_cmd_out, f_out, sat_out = np.zeros(n), np.zeros(n), np.zeros(n, dtype=bool)
    x = np.array([v_cyc[0], 0.0])  # state: speed, distance
    integral = 0.0

    for k in range(n):
        v, s = float(x[0]), float(x[1])
        g = grade_at(s)
        v_out[k], s_out[k], g_out[k] = v, s, g
        if k == n - 1:
            break

        e = v_ref[k] - v
        f_ff = (
            m * a_ref[k] + float(physics.resistance_force_n(p, v_ref[k], g))
            if cfg.feedforward
            else 0.0
        )
        f_cmd = f_ff + kp * e + ki * integral
        f_max = max_drive_force_n(p, v)
        f_min = -cfg.max_brake_decel_mps2 * p.body.mass_kg
        if v_ref[k] == 0.0 and v < 0.05:
            f_cmd = min(f_cmd, 0.0)  # standing still: hold with the brakes, do not creep
        f_app = min(max(f_cmd, f_min), f_max)
        saturated = f_app != f_cmd

        # conditional integration (anti-windup)
        if (
            not cfg.anti_windup
            or not saturated
            or (f_cmd > f_max and e < 0)
            or (f_cmd < f_min and e > 0)
        ):
            integral += e * h

        def rhs(_t, xs, f_app=f_app):
            vv = max(xs[0], 0.0)
            gg = grade_at(xs[1])
            f_res = float(physics.resistance_force_n(p, vv, gg))
            if vv <= 0.0 and f_app <= f_res:  # brakes/static friction hold the car
                return np.array([0.0, 0.0])
            return np.array([(f_app - f_res) / m, vv])

        x = step(rhs, t[k], x, h)
        x[0] = max(x[0], 0.0)
        f_cmd_out[k], f_out[k], sat_out[k] = f_cmd, f_app, saturated

    achieved = DriveCycle(f"{cycle.name} (forward)", t, v_out, g_out)
    energy = simulate_cycle(p, achieved)
    err = (v_out - v_ref) * 3.6
    summary = {
        "speed_rmse_kmh": float(np.sqrt(np.mean(err**2))),
        "speed_max_abs_error_kmh": float(np.max(np.abs(err))),
        "max_overshoot_kmh": float(np.max(err)),
        "distance_km": float(s_out[-1] / 1000.0),
        "distance_ref_km": float(s_ref[-1] / 1000.0),
        "saturated_fraction": float(sat_out[:-1].mean()),
        "battery_energy_kwh": energy.summary["battery_energy_kwh"],
        "consumption_kwh_per_100km": energy.summary["consumption_kwh_per_100km"],
    }
    return ForwardResult(t, v_ref, v_out, s_out, g_out, f_cmd_out, f_out, sat_out, energy, summary)
