"""DC fast charging simulation: charger + battery management (BMS) + battery + cooling.

The problem
-----------
How long does a charge from 10 % to 80 % take, what limits the current at
each moment, and does the charge strategy always respect the battery's
voltage, current and temperature limits?

Charge strategy (what a BMS does)
---------------------------------
At every time step the BMS allows the smallest of three currents:

    I_CC = f(T) * g(SOC) * C_max * Q         current limit (C-rate), reduced by temperature and SOC
    I_CP = current for P_charger              charging station power limit
    I_CV = (U_max - (OCV - U1)) / R0          current that keeps U_terminal at the voltage limit

The active limit names the phase:
    CC   constant current        (current limit active)
    CC*  derated CC              (current limit active and f(T) * g(SOC) < 1)
    CP   constant power          (charger limit active)
    CV   constant voltage        (voltage limit active, current tapers)

Charging stops when the target SOC is reached, when the current in the CV
phase falls below the end current (C/20), when the battery is too hot, or at
the time limit.

The battery itself is the same equivalent-circuit and thermal model used for
driving (battery.step), with an extra active-cooling term:
    Q_cooling = cooling_w_k * max(T - T_coolant, 0)

Sign convention as everywhere: battery current > 0 = discharge. Charging
current is reported as a positive number ``charge_current_a`` for readability.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field

import numpy as np
import pandas as pd

from evdynasim import battery
from evdynasim.params import ModelParams, with_changes

J_TO_KWH = 1.0 / 3.6e6
PHASES = ("CC", "CC*", "CP", "CV")


@dataclass
class ChargeResult:
    series: pd.DataFrame  # one row per time step
    summary: dict = field(default_factory=dict)

    def time_to_soc_min(self, soc: float) -> float:
        """Minutes from the start until the SOC first reaches ``soc`` (NaN if never)."""
        s = self.series
        reached = s.index[s["soc_end"] >= soc]
        return float(s.loc[reached[0], "t_end_s"] / 60.0) if len(reached) else math.nan


def derate_factor(p: ModelParams, state: battery.BatteryState) -> float:
    """BMS derating of the C-rate: temperature factor times SOC factor."""
    return p.charging.temperature_factor(state.temp_c) * p.charging.soc_factor(state.soc)


def charge_limits(p: ModelParams, state: battery.BatteryState) -> dict[str, float]:
    """The three current limits (A, positive = charging) at the present battery state."""
    ch, bat = p.charging, p.battery
    ocv = float(bat.ocv_pack_v(state.soc))
    r0 = float(bat.r0_pack_ohm(state.temp_c))
    e = ocv - state.u1_v
    v_max = ch.cv_cell_voltage_v * bat.cells_series

    i_cc = derate_factor(p, state) * ch.max_c_rate * bat.capacity_ah
    i_cp = -battery.current_for_power(e, r0, -ch.charger_max_power_w)[0]
    if r0 > 0.0:
        i_cv = (v_max - e) / r0
    else:
        i_cv = math.inf if e < v_max else 0.0
    return {"CC": max(i_cc, 0.0), "CP": max(i_cp, 0.0), "CV": max(i_cv, 0.0)}


def simulate_charging(
    p: ModelParams,
    soc_start: float = 0.10,
    soc_target: float = 0.80,
    ambient_temp_c: float | None = None,
    initial_temp_c: float | None = None,
    charger_power_kw: float | None = None,
    dt_s: float = 1.0,
    max_time_s: float = 4 * 3600.0,
) -> ChargeResult:
    """Simulate one charging session.

    ambient_temp_c   outside temperature (default: the configuration value)
    initial_temp_c   battery temperature at plug-in (default: equal to ambient,
                     i.e. a vehicle that has been parked; a higher value models
                     a preconditioned or just-driven battery)
    charger_power_kw overrides the charging station power
    """
    if p.charging is None:
        raise ValueError("configuration has no 'charging' section")
    if not 0.0 <= soc_start < soc_target <= 1.0:
        raise ValueError("need 0 <= soc_start < soc_target <= 1")

    amb = p.env.ambient_temp_c if ambient_temp_c is None else ambient_temp_c
    t0 = amb if initial_temp_c is None else initial_temp_c
    p = with_changes(
        p,
        {"env.ambient_temp_c": amb, "thermal.initial_temp_c": t0, "battery.initial_soc": soc_start},
    )
    if charger_power_kw is not None:
        p = dataclasses.replace(
            p, charging=dataclasses.replace(p.charging, charger_max_power_w=charger_power_kw * 1e3)
        )
    ch, bat = p.charging, p.battery
    i_end = ch.end_current_c_rate * bat.capacity_ah

    state = battery.initial_state(p)
    rows = []
    stop_reason = "time limit"
    t = 0.0
    while t < max_time_s:
        if state.soc >= soc_target:
            stop_reason = "target SOC reached"
            break
        if state.temp_c >= ch.max_cell_temp_c:
            stop_reason = "battery temperature limit"
            break

        limits = charge_limits(p, state)
        phase = min(limits, key=limits.get)
        current = limits[phase]
        if phase == "CC" and derate_factor(p, state) < 1.0:
            phase = "CC*"
        if phase == "CV" and current < i_end:
            stop_reason = "end of CV phase (current below C/20)"
            break
        if current <= 0.0:
            stop_reason = "no charging current allowed"
            break

        ocv = float(bat.ocv_pack_v(state.soc))
        r0 = float(bat.r0_pack_ohm(state.temp_c))
        p_terminal = -(ocv - state.u1_v + r0 * current) * current  # negative = into the battery
        cooling = ch.cooling_w_k * max(state.temp_c - ch.coolant_temp_c, 0.0)

        new_state, info = battery.step(p, state, p_terminal, dt_s, extra_heat_removal_w=cooling)
        p_in = -info.terminal_v * info.current_a  # power into the battery terminals (> 0)
        rows.append(
            {
                "t_end_s": t + dt_s,
                "dt_s": dt_s,
                "phase": phase,
                "charge_current_a": -info.current_a,
                "terminal_v": info.terminal_v,
                "ocv_v": info.ocv_v,
                "power_in_kw": p_in / 1e3,
                "grid_power_kw": p_in / ch.charger_efficiency / 1e3,
                "battery_loss_kw": info.loss_w / 1e3,
                "stored_power_kw": -info.ocv_v * info.current_a / 1e3,
                "cooling_kw": cooling / 1e3,
                "limit_cc_a": limits["CC"],
                "limit_cp_a": limits["CP"],
                "limit_cv_a": limits["CV"],
                "soc_start": state.soc,
                "soc_end": new_state.soc,
                "temp_end_c": new_state.temp_c,
            }
        )
        state = new_state
        t += dt_s

    s = pd.DataFrame(rows)
    if s.empty:
        return ChargeResult(s, {"stop_reason": stop_reason, "duration_min": 0.0})

    dt = s["dt_s"].to_numpy()

    def kwh(col: str) -> float:
        return float(np.sum(s[col].to_numpy() * 1e3 * dt) * J_TO_KWH)

    e_in, e_grid, e_stored, e_loss = (
        kwh("power_in_kw"),
        kwh("grid_power_kw"),
        kwh("stored_power_kw"),
        kwh("battery_loss_kw"),
    )
    v_max = ch.cv_cell_voltage_v * bat.cells_series
    summary = {
        "stop_reason": stop_reason,
        "duration_min": float(s["t_end_s"].iloc[-1] / 60.0),
        "soc_start": soc_start,
        "soc_end": float(s["soc_end"].iloc[-1]),
        "ambient_temp_c": amb,
        "initial_temp_c": t0,
        "charger_power_kw": ch.charger_max_power_w / 1e3,
        "energy_grid_kwh": e_grid,
        "energy_into_battery_kwh": e_in,
        "energy_stored_kwh": e_stored,
        "charger_loss_kwh": e_grid - e_in,
        "battery_loss_kwh": e_loss,
        "efficiency_grid_to_cells": e_stored / e_grid,
        "average_power_kw": float(e_in / (s["t_end_s"].iloc[-1] / 3600.0)),
        "peak_power_kw": float(s["power_in_kw"].max()),
        "max_current_a": float(s["charge_current_a"].max()),
        "max_terminal_v": float(s["terminal_v"].max()),
        "voltage_limit_v": v_max,
        "max_temp_c": float(max(s["temp_end_c"].max(), t0)),
        "final_temp_c": float(s["temp_end_c"].iloc[-1]),
    }
    for ph in PHASES:
        summary[f"time_{ph.replace('*', '_derated')}_min"] = float(
            dt[s["phase"] == ph].sum() / 60.0
        )
    return ChargeResult(s, summary)


# ---- studies ----------------------------------------------------------------------------
def ambient_study(p: ModelParams, temps_c=(-10, 0, 10, 25, 35), soc_start=0.10, soc_target=0.80):
    """Charge time vs ambient temperature, parked (battery at ambient) vs preconditioned to 25 degC."""
    rows = []
    for amb in temps_c:
        for label, t0 in (("parked", amb), ("preconditioned", 25.0)):
            r = simulate_charging(p, soc_start, soc_target, ambient_temp_c=amb, initial_temp_c=t0)
            rows.append({"ambient_c": amb, "battery_start": label,
                         "time_min": r.summary["duration_min"],
                         "max_temp_c": r.summary["max_temp_c"],
                         "stop_reason": r.summary["stop_reason"]})  # fmt: skip
    return pd.DataFrame(rows)


def charger_power_study(
    p: ModelParams, powers_kw=(50, 100, 150, 250, 350), soc_start=0.10, soc_target=0.80
):
    """Charge time vs charging station power. Above the battery's own limit, more power does not help."""
    rows = []
    for pw in powers_kw:
        r = simulate_charging(p, soc_start, soc_target, charger_power_kw=pw)
        rows.append({"charger_kw": pw, "time_min": r.summary["duration_min"],
                     "peak_power_kw": r.summary["peak_power_kw"],
                     "average_power_kw": r.summary["average_power_kw"]})  # fmt: skip
    return pd.DataFrame(rows)
