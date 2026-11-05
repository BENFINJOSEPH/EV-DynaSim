"""Backward-facing (inverse) vehicle simulation.

Given the speed profile, compute what the powertrain and battery must
deliver. Speed is an input; vehicle motion is not solved.

Structure
---------
1. Vehicle + powertrain, vectorised over all intervals (no state involved).
2. Battery, sequential loop (state: SOC, RC voltage, temperature). The loop
   also applies the SOC-dependent charge power limit, which can curtail regen.

Discretisation: N samples -> N-1 intervals. For interval k
    a_k = (v_{k+1} - v_k) / dt_k,   vbar_k = (v_k + v_{k+1}) / 2
With this choice the inertial work is exactly the change of kinetic energy,
so the energy balance closes to machine precision (see tests).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd
from numpy.typing import ArrayLike, NDArray

from evdynasim import battery, physics
from evdynasim.cycles import DriveCycle
from evdynasim.params import KMH_TO_MPS, ModelParams
from evdynasim.powertrain import compute_powertrain

J_TO_KWH = 1.0 / 3.6e6

#: Energy sinks: together they must equal the chemical energy taken from the cells.
ENERGY_SINKS = (
    "aero_kwh", "rolling_kwh", "grade_kwh", "kinetic_change_kwh", "gearbox_loss_kwh",
    "motor_loss_kwh", "friction_brake_kwh", "aux_kwh", "battery_loss_kwh",
)  # fmt: skip


def steady_state_power_w(p: ModelParams, speed_kmh: ArrayLike) -> NDArray[np.float64]:
    """DC power demand at the battery terminals at constant speed, flat road."""
    v = np.atleast_1d(np.asarray(speed_kmh, dtype=float)) * KMH_TO_MPS
    pt = compute_powertrain(p, physics.traction_force_n(p, v), v)
    out = pt.motor_elec_w + p.aux.power_w(p.env.ambient_temp_c)
    return out if np.ndim(speed_kmh) else out[0]


@dataclass
class SimulationResult:
    """Interval time series (length N-1) and sample states (length N)."""

    cycle_name: str
    vehicle_name: str
    series: pd.DataFrame  # one row per interval, SI units
    soc: NDArray[np.float64]  # N samples
    temp_c: NDArray[np.float64]  # N samples
    summary: dict[str, float] = field(default_factory=dict)

    def __getattr__(self, name: str):
        # convenient access: res.battery_power_w instead of res.series["battery_power_w"]
        series = self.__dict__.get("series")
        if series is not None and name in series.columns:
            return series[name].to_numpy()
        raise AttributeError(name)

    def to_dataframe(self) -> pd.DataFrame:
        """Per-interval results in engineering units, ready for CSV/plots."""
        s = self.series
        return pd.DataFrame(
            {
                "t_s": s["t_start_s"],
                "dt_s": s["dt_s"],
                "speed_kmh": s["speed_mps"] * 3.6,
                "accel_mps2": s["accel_mps2"],
                "grade_pct": s["grade_pct"],
                "motor_speed_rpm": s["motor_speed_rads"] * 60 / (2 * np.pi),
                "motor_torque_nm": s["motor_torque_nm"],
                "p_wheel_kw": s["wheel_power_w"] / 1e3,
                "p_battery_kw": s["battery_power_w"] / 1e3,
                "p_friction_brake_kw": s["friction_brake_w"] / 1e3,
                "current_a": s["current_a"],
                "terminal_v": s["terminal_v"],
                "soc_end": self.soc[1:],
                "temp_end_c": self.temp_c[1:],
                "torque_limited": s["torque_limited"],
                "battery_limited": s["battery_limited"],
            }
        )


def _kwh(power_w, dt_s) -> float:
    return float(np.sum(np.asarray(power_w) * dt_s) * J_TO_KWH)


def simulate_cycle(p: ModelParams, cycle: DriveCycle) -> SimulationResult:
    """Run the backward-facing simulation of ``p`` over ``cycle``."""
    t, v = cycle.time_s, cycle.speed_mps
    dt = np.diff(t)
    vbar = 0.5 * (v[:-1] + v[1:])
    accel = np.diff(v) / dt
    grade = 0.5 * (cycle.grade_pct[:-1] + cycle.grade_pct[1:])

    # 1. vehicle and powertrain (vectorised) ------------------------------------
    f_aero = physics.aero_drag_n(p, vbar)
    f_roll = physics.rolling_resistance_n(p, vbar, grade)
    f_grade = np.where(vbar > 0.0, physics.grade_force_n(p, grade), 0.0)  # no hill-hold energy
    f_inertia = physics.inertial_force_n(p, accel)
    f_wheel = f_aero + f_roll + f_grade + f_inertia
    pt = compute_powertrain(p, f_wheel, vbar)

    motor_elec = pt.motor_elec_w.copy()
    motor_loss = pt.motor_loss_w.copy()
    gearbox_loss = pt.gearbox_loss_w.copy()
    friction = pt.friction_brake_w.copy()
    regen_wheel = pt.regen_wheel_w.copy()
    aux_w = p.aux.power_w(p.env.ambient_temp_c)

    # 2. battery (sequential) ---------------------------------------------------
    n = len(dt)
    current = np.empty(n)
    terminal_v = np.empty(n)
    batt_power = np.empty(n)
    batt_loss = np.empty(n)
    chem_power = np.empty(n)
    batt_limited = np.zeros(n, dtype=bool)
    soc = np.empty(n + 1)
    temp = np.empty(n + 1)
    curtailed_j = 0.0

    state = battery.initial_state(p)
    soc[0], temp[0] = state.soc, state.temp_c
    for k in range(n):
        p_req = motor_elec[k] + aux_w
        limit = p.battery.charge_limit_w(state.soc)
        if p_req < -limit and motor_elec[k] < 0.0:
            # battery cannot accept all regen: scale regen down, rest to friction brakes
            s = max(0.0, 1.0 - (-limit - p_req) / (-motor_elec[k]))
            curtailed_j += (1.0 - s) * regen_wheel[k] * dt[k]
            friction[k] += (1.0 - s) * regen_wheel[k]
            regen_wheel[k] *= s
            motor_elec[k] *= s
            motor_loss[k] *= s
            gearbox_loss[k] *= s
            p_req = motor_elec[k] + aux_w

        state_next, info = battery.step(p, state, p_req, dt[k])
        current[k] = info.current_a
        terminal_v[k] = info.terminal_v
        batt_power[k] = info.terminal_v * info.current_a  # == p_req unless beyond max power
        batt_loss[k] = info.loss_w
        chem_power[k] = info.ocv_v * info.current_a
        batt_limited[k] = info.power_limited
        state = state_next
        soc[k + 1], temp[k + 1] = state.soc, state.temp_c

    series = pd.DataFrame(
        {
            "t_start_s": t[:-1],
            "dt_s": dt,
            "speed_mps": vbar,
            "accel_mps2": accel,
            "grade_pct": grade,
            "f_aero_n": f_aero,
            "f_roll_n": f_roll,
            "f_grade_n": f_grade,
            "f_inertia_n": f_inertia,
            "wheel_power_w": f_wheel * vbar,
            "motor_speed_rads": pt.motor_speed_rads,
            "motor_torque_nm": pt.motor_torque_nm,
            "motor_elec_w": motor_elec,
            "motor_loss_w": motor_loss,
            "gearbox_loss_w": gearbox_loss,
            "friction_brake_w": friction,
            "regen_wheel_w": regen_wheel,
            "aux_w": np.full(n, aux_w),
            "battery_power_w": batt_power,
            "battery_loss_w": batt_loss,
            "chemical_power_w": chem_power,
            "current_a": current,
            "terminal_v": terminal_v,
            "torque_limited": pt.torque_limited,
            "battery_limited": batt_limited,
        }
    )

    distance_km = float(np.sum(vbar * dt) / 1000.0)
    e_batt = _kwh(batt_power, dt)
    summary = {
        "duration_s": float(t[-1] - t[0]),
        "distance_km": distance_km,
        "mean_speed_kmh": distance_km / ((t[-1] - t[0]) / 3600.0),
        "max_speed_kmh": float(v.max() * 3.6),
        "battery_energy_kwh": e_batt,
        "consumption_kwh_per_100km": 100.0 * e_batt / distance_km if distance_km > 0 else np.nan,
        "chemical_energy_kwh": _kwh(chem_power, dt),
        "regen_energy_kwh": _kwh(np.maximum(-motor_elec, 0.0), dt),
        "regen_curtailed_kwh": curtailed_j * J_TO_KWH,
        "aero_kwh": _kwh(f_aero * vbar, dt),
        "rolling_kwh": _kwh(f_roll * vbar, dt),
        "grade_kwh": _kwh(f_grade * vbar, dt),
        "kinetic_change_kwh": _kwh(f_inertia * vbar, dt),
        "gearbox_loss_kwh": _kwh(gearbox_loss, dt),
        "motor_loss_kwh": _kwh(motor_loss, dt),
        "friction_brake_kwh": _kwh(friction, dt),
        "aux_kwh": _kwh(np.full(n, aux_w), dt),
        "battery_loss_kwh": _kwh(batt_loss, dt),
        "peak_battery_power_kw": float(batt_power.max() / 1e3),
        "peak_regen_power_kw": float(max(0.0, -batt_power.min()) / 1e3),
        "max_current_a": float(np.abs(current).max()),
        "min_terminal_voltage_v": float(terminal_v.min()),
        "initial_soc": float(soc[0]),
        "final_soc": float(soc[-1]),
        "max_temp_c": float(temp.max()),
        "final_temp_c": float(temp[-1]),
        "torque_limited_steps": int(pt.torque_limited.sum()),
        "battery_limited_steps": int(batt_limited.sum()),
    }
    return SimulationResult(cycle.name, p.name, series, soc, temp, summary)
