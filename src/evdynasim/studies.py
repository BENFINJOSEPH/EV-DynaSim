"""Engineering studies built on the simulator: sensitivity, ambient temperature, runtime."""

from __future__ import annotations

import dataclasses
import time
from pathlib import Path

import numpy as np
import pandas as pd

from evdynasim.cycles import DriveCycle
from evdynasim.interp import Table2D
from evdynasim.params import ModelParams, get_value, with_changes
from evdynasim.simulate import simulate_cycle

SENSITIVITY_PARAMS = {
    "body.mass_kg": "Vehicle mass",
    "body.drag_coefficient": "Drag coefficient Cd",
    "body.rolling_coefficient": "Rolling resistance Crr",
    "motor.loss_scale": "Motor + inverter losses",
    "aux.base_power_w": "Auxiliary base power",
    "brakes.regen_fraction": "Regen share of braking",
}


def _scaled(p: ModelParams, key: str, factor: float) -> ModelParams:
    if key == "motor.loss_scale":
        lm = p.motor.loss_map
        if lm is None:  # constant-efficiency model: scale the loss fraction instead
            eta = p.motor.constant_efficiency
            return with_changes(p, {"motor.constant_efficiency": 1 / (1 + (1 / eta - 1) * factor)})
        motor = dataclasses.replace(p.motor, loss_map=Table2D(lm.x, lm.y, lm.z * factor))
        return dataclasses.replace(p, motor=motor)
    return with_changes(p, {key: get_value(p, key) * factor})


def sensitivity(p: ModelParams, cycle: DriveCycle, rel: float = 0.10) -> pd.DataFrame:
    """One-at-a-time sensitivity of consumption to +/- rel change of each parameter."""
    base = simulate_cycle(p, cycle).summary["consumption_kwh_per_100km"]
    rows = []
    for key, label in SENSITIVITY_PARAMS.items():
        lo = simulate_cycle(_scaled(p, key, 1 - rel), cycle).summary["consumption_kwh_per_100km"]
        hi = simulate_cycle(_scaled(p, key, 1 + rel), cycle).summary["consumption_kwh_per_100km"]
        rows.append({"parameter": label, "key": key, "base": base,
                     "minus_pct": 100 * (lo - base) / base, "plus_pct": 100 * (hi - base) / base})  # fmt: skip
    df = pd.DataFrame(rows)
    return df.reindex(df[["minus_pct", "plus_pct"]].abs().max(axis=1).sort_values().index[::-1])


def ambient_sweep(
    p: ModelParams, cycle: DriveCycle, temps_c=(-15, -5, 5, 15, 25, 35)
) -> pd.DataFrame:
    """Consumption vs ambient temperature (vehicle soaked at ambient before the drive)."""
    rows = []
    for t_amb in temps_c:
        q = with_changes(
            p, {"env.ambient_temp_c": float(t_amb), "thermal.initial_temp_c": float(t_amb)}
        )
        s = simulate_cycle(q, cycle).summary
        d = s["distance_km"]
        rows.append({
            "ambient_c": t_amb,
            "consumption_kwh_per_100km": s["consumption_kwh_per_100km"],
            "driving_kwh_100km": 100 * (s["aero_kwh"] + s["rolling_kwh"] + s["grade_kwh"]
                                        + s["kinetic_change_kwh"] + s["friction_brake_kwh"]) / d,
            "powertrain_loss_kwh_100km": 100 * (s["gearbox_loss_kwh"] + s["motor_loss_kwh"]) / d,
            "aux_hvac_kwh_100km": 100 * s["aux_kwh"] / d,
            "battery_loss_kwh_100km": 100 * s["battery_loss_kwh"] / d,
        })  # fmt: skip
    return pd.DataFrame(rows)


def benchmark(
    p: ModelParams, config_path: Path, cycle: DriveCycle, repeat: int = 20
) -> pd.DataFrame:
    """Runtime of the Python and (if built) the C++ engine on the same cycle."""
    from evdynasim import cpp_engine

    def timed(fn, n):
        fn()  # warm-up
        t0 = time.perf_counter()
        for _ in range(n):
            fn()
        return (time.perf_counter() - t0) / n

    sim_seconds = cycle.duration_s
    rows = [("Python (NumPy + loop)", timed(lambda: simulate_cycle(p, cycle), repeat))]
    if cpp_engine.is_available():
        # includes reading the config file and copying arrays through ctypes
        rows.append(("C++ via C API (ctypes)",
                     timed(lambda: cpp_engine.simulate_cpp(config_path, cycle), repeat * 10)))  # fmt: skip
    df = pd.DataFrame(rows, columns=["engine", "seconds_per_run"])
    df["ms_per_run"] = df.seconds_per_run * 1e3
    df["realtime_factor"] = sim_seconds / df.seconds_per_run
    df["speedup_vs_python"] = df.seconds_per_run.iloc[0] / df.seconds_per_run
    return df


def motorway_cycle(speed_kmh: float = 120.0, duration_s: float = 1800.0) -> DriveCycle:
    """Accelerate, hold a motorway speed, stop."""
    t = np.arange(0.0, duration_s + 1)
    a = 1.5  # m/s^2 for the acceleration and the final stop
    v = np.minimum.reduce([t * a, np.full_like(t, speed_kmh / 3.6), (duration_s - t) * a])
    return DriveCycle(f"motorway {speed_kmh:g} km/h", t, np.clip(v, 0, None), np.zeros_like(t))
