"""Engineering plots. Kept separate so the simulation core has no GUI dependency."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # file output only; works on CI machines without a display
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from evdynasim.params import ModelParams  # noqa: E402
from evdynasim.simulate import SimulationResult, steady_state_power_w  # noqa: E402


def plot_steady_state(p: ModelParams, out_file: Path, v_max_kmh: float = 160.0) -> Path:
    speeds = np.linspace(0.0, v_max_kmh, 161)
    power_kw = steady_state_power_w(p, speeds) / 1e3
    with np.errstate(divide="ignore", invalid="ignore"):
        cons = np.where(speeds > 0, power_kw / speeds * 100.0, np.nan)  # kWh/100 km

    fig, ax1 = plt.subplots(figsize=(9, 5))
    ax1.plot(speeds, power_kw, lw=2, label="Battery power")
    ax1.set_xlabel("Vehicle speed (km/h)")
    ax1.set_ylabel("Battery power (kW)")
    ax1.grid(True, alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(speeds, cons, lw=2, ls="--", color="tab:orange", label="Consumption")
    ax2.set_ylabel("Consumption (kWh/100 km)")
    ax2.set_ylim(0, 40)
    fig.legend(loc="upper left", bbox_to_anchor=(0.1, 0.9))
    ax1.set_title(f"Steady-state demand, flat road - {p.name}")
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file


def plot_cycle(res: SimulationResult, out_file: Path) -> Path:
    t = res.t_start_s
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    axes[0].plot(t, res.speed_mps * 3.6, lw=1.5)
    axes[0].set_ylabel("Speed (km/h)")

    axes[1].plot(t, res.battery_power_w / 1e3, lw=1.2, label="Battery")
    axes[1].plot(t, res.wheel_power_w / 1e3, lw=1.0, alpha=0.7, label="Wheel")
    axes[1].fill_between(
        t, 0, -res.friction_brake_w / 1e3, color="tab:red", alpha=0.3, label="Friction brake"
    )
    axes[1].axhline(0, color="k", lw=0.5)
    axes[1].set_ylabel("Power (kW)")
    axes[1].legend(loc="upper left", ncol=3)

    axes[2].plot(np.append(t, t[-1] + res.dt_s[-1]), res.soc * 100.0, lw=1.5)
    axes[2].set_ylabel("SOC (%)")
    axes[2].set_xlabel("Time (s)")

    for ax in axes:
        ax.grid(True, alpha=0.3)
    s = res.summary
    fig.suptitle(
        f"{res.cycle_name}: {s['distance_km']:.2f} km, "
        f"{s['consumption_kwh_per_100km']:.1f} kWh/100 km - {res.vehicle_name}"
    )
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file


def plot_energy_breakdown(res: SimulationResult, out_file: Path) -> Path:
    s = res.summary
    labels = ["Aero drag", "Rolling", "Grade (net)", "Gearbox loss", "Motor+inverter loss",
              "Friction brake", "Auxiliaries", "Battery loss"]  # fmt: skip
    keys = ["aero_kwh", "rolling_kwh", "grade_kwh", "gearbox_loss_kwh", "motor_loss_kwh",
            "friction_brake_kwh", "aux_kwh", "battery_loss_kwh"]  # fmt: skip
    values = [s[k] for k in keys]

    fig, ax = plt.subplots(figsize=(9, 4.5))
    bars = ax.barh(labels, values)
    ax.bar_label(bars, fmt="%.3f kWh", padding=3)
    ax.invert_yaxis()
    ax.set_xlabel("Energy (kWh)")
    ax.set_title(
        f"Where the battery energy goes - {res.cycle_name} "
        f"(cells: {s['chemical_energy_kwh']:.3f} kWh)"
    )
    ax.grid(True, axis="x", alpha=0.3)
    ax.set_xlim(min(0.0, min(values) * 1.3), max(values) * 1.3)
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file


def plot_tornado(df_by_cycle: dict, out_file: Path) -> Path:
    """Side-by-side tornado charts, one per cycle (dict: cycle name -> sensitivity table)."""
    fig, axes = plt.subplots(1, len(df_by_cycle), figsize=(6 * len(df_by_cycle), 4.2), sharey=False)
    axes = np.atleast_1d(axes)
    for ax, (name, df) in zip(axes, df_by_cycle.items(), strict=True):
        df = df.iloc[::-1]
        y = np.arange(len(df))
        ax.barh(y, df["minus_pct"], color="tab:blue", label="-10 %")
        ax.barh(y, df["plus_pct"], color="tab:orange", label="+10 %")
        ax.set_yticks(y, df["parameter"])
        ax.axvline(0, color="k", lw=0.8)
        ax.set_xlabel("Change in consumption (%)")
        ax.set_title(f"{name}: {df['base'].iloc[0]:.1f} kWh/100 km")
        ax.grid(True, axis="x", alpha=0.3)
        ax.legend(loc="lower right", fontsize=8)
    fig.suptitle("Sensitivity of energy consumption to +/-10 % parameter changes")
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file


def plot_ambient(df, cycle_name: str, out_file: Path) -> Path:
    parts = [("driving_kwh_100km", "Driving resistances + brakes"),
             ("powertrain_loss_kwh_100km", "Motor + gearbox losses"),
             ("aux_hvac_kwh_100km", "Auxiliaries + HVAC"),
             ("battery_loss_kwh_100km", "Battery losses (inside the pack)")]  # fmt: skip
    fig, ax = plt.subplots(figsize=(9, 4.8))
    x = np.arange(len(df))
    bottom = np.zeros(len(df))
    for col, label in parts:
        ax.bar(x, df[col], bottom=bottom, label=label)
        bottom += df[col].to_numpy()
    ax.plot(x, df["consumption_kwh_per_100km"], "k.", ms=1)
    ax.set_xticks(x, [f"{t} degC" for t in df["ambient_c"]])
    ax.set_ylabel("kWh/100 km")
    ax.set_title(f"Consumption vs ambient temperature - {cycle_name}")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file


def plot_forward(
    results: dict,
    out_file: Path,
    window=(1000, 1180),
    title: str = "Forward simulation: PI driver with a 25 kW motor (EUDC part of the NEDC)",
) -> Path:
    """Speed tracking and drive force for several forward runs (dict: label -> ForwardResult)."""
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    first = next(iter(results.values()))
    t = first.time_s
    sel = (t >= window[0]) & (t <= window[1])
    axes[0].plot(t[sel], first.speed_ref_mps[sel] * 3.6, "k--", lw=1.2, label="reference")
    for label, r in results.items():
        axes[0].plot(t[sel], r.speed_mps[sel] * 3.6, lw=1.3, label=label)
        axes[1].plot(t[sel], r.force_n[sel] / 1e3, lw=1.0, label=label)
    axes[0].set_ylabel("Speed (km/h)")
    axes[1].set_ylabel("Applied wheel force (kN)")
    axes[1].set_xlabel("Time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="upper left")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file


def plot_motor_map(p: ModelParams, res: SimulationResult, out_file: Path) -> Path:
    """Efficiency map with the torque envelope and the operating points of a cycle."""
    from evdynasim.powertrain import motor_loss_w

    speed = np.linspace(100, p.motor.max_speed_rads * 60 / (2 * np.pi), 120)
    torque = np.linspace(5, p.motor.max_torque_nm, 100)
    s_grid, t_grid = np.meshgrid(speed, torque)
    w = s_grid * 2 * np.pi / 60
    p_mech = t_grid * w
    eta = p_mech / (p_mech + motor_loss_w(p, w, t_grid, True))
    envelope = p.motor.max_torque_at(w[0])
    eta = np.where(t_grid <= envelope, eta, np.nan)

    fig, ax = plt.subplots(figsize=(9, 5.5))
    cs = ax.contourf(s_grid, t_grid, eta * 100, levels=[70, 80, 85, 88, 90, 92, 93, 94, 95, 96, 97, 98],
                     cmap="viridis", extend="both")  # fmt: skip
    fig.colorbar(cs, ax=ax, label="Motor + inverter efficiency (%)")
    ax.plot(speed, envelope, "k", lw=2, label="torque envelope")
    rpm = res.motor_speed_rads * 60 / (2 * np.pi)
    tq = np.abs(res.motor_torque_nm)
    ax.plot(rpm[tq > 0], tq[tq > 0], ".", color="tab:red", ms=3, alpha=0.6,
            label=f"operating points ({res.cycle_name})")  # fmt: skip
    ax.set_xlabel("Motor speed (rpm)")
    ax.set_ylabel("|Torque| (Nm)")
    ax.set_ylim(0, p.motor.max_torque_nm * 1.05)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Motor efficiency map (from the synthetic loss map)")
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file


_PHASE_COLORS = {"CC": "#d9ead3", "CC*": "#fff2cc", "CP": "#cfe2f3", "CV": "#f4cccc"}


def plot_charging_session(res, out_file: Path, title: str = "") -> Path:
    """Current with its three limits, voltage, power, SOC and temperature; phases shaded."""
    s = res.series
    t = s["t_end_s"].to_numpy() / 60.0
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    # phase shading
    phase = s["phase"].to_numpy()
    start = 0
    for k in range(1, len(phase) + 1):
        if k == len(phase) or phase[k] != phase[start]:
            for ax in axes:
                ax.axvspan(t[start] - s["dt_s"].iloc[0] / 60, t[k - 1], color=_PHASE_COLORS[phase[start]],
                           lw=0)  # fmt: skip
            start = k

    ax = axes[0]
    ax.plot(t, s["charge_current_a"], "k", lw=2, label="charge current")
    ax.plot(t, s["limit_cc_a"], "--", lw=1, label="C-rate limit (derated)")
    ax.plot(t, s["limit_cp_a"], "--", lw=1, label="charger power limit")
    ax.plot(t, np.minimum(s["limit_cv_a"], 1.5 * s["charge_current_a"].max()), "--", lw=1,
            label="voltage limit")  # fmt: skip
    ax.set_ylabel("Current (A)")
    ax.set_ylim(0, 1.3 * s["charge_current_a"].max())
    ax.legend(fontsize=8, loc="lower left", ncol=2)

    ax = axes[1]
    ax.plot(t, s["terminal_v"], lw=1.5, label="terminal voltage")
    ax.plot(t, s["ocv_v"], lw=1.0, alpha=0.7, label="open-circuit voltage")
    ax.axhline(res.summary["voltage_limit_v"], color="tab:red", ls=":", label="CV limit")
    ax.set_ylabel("Voltage (V)")
    ax.legend(fontsize=8, loc="lower right")

    ax = axes[2]
    ax.plot(t, s["grid_power_kw"], lw=1.0, label="from grid")
    ax.plot(t, s["power_in_kw"], lw=1.5, label="into battery")
    ax.set_ylabel("Power (kW)")
    ax.legend(fontsize=8, loc="lower left")

    ax = axes[3]
    ax.plot(t, s["soc_end"] * 100, lw=1.5, color="tab:green", label="SOC")
    ax.set_ylabel("SOC (%)")
    ax2 = ax.twinx()
    ax2.plot(t, s["temp_end_c"], lw=1.5, color="tab:red", label="battery temperature")
    ax2.set_ylabel("Temperature (degC)")
    ax.set_xlabel("Time (min)")
    ax.legend(fontsize=8, loc="upper left")
    ax2.legend(fontsize=8, loc="lower right")

    for ax in axes:
        ax.grid(True, alpha=0.3)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in _PHASE_COLORS.values()]
    fig.legend(handles, ["CC: current limit", "CC*: derated current limit", "CP: charger power limit",
                         "CV: voltage limit"], loc="upper center", ncol=4, fontsize=8,
               bbox_to_anchor=(0.5, 0.965))  # fmt: skip
    sm = res.summary
    fig.suptitle(title or (f"DC charging {sm['soc_start']:.0%} -> {sm['soc_end']:.0%}: "
                           f"{sm['duration_min']:.1f} min, {sm['charger_power_kw']:.0f} kW charger, "
                           f"{sm['ambient_temp_c']:.0f} degC"))  # fmt: skip
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_file, dpi=140)
    plt.close(fig)
    return out_file


def plot_charging_curves(runs: dict, out_file: Path) -> Path:
    """Charging power vs SOC for several sessions (dict: label -> ChargeResult)."""
    fig, ax = plt.subplots(figsize=(9, 5))
    for label, r in runs.items():
        s = r.series
        ax.plot(s["soc_end"] * 100, s["power_in_kw"], lw=1.8,
                label=f"{label}: {r.summary['duration_min']:.0f} min")  # fmt: skip
    ax.set_xlabel("SOC (%)")
    ax.set_ylabel("Charging power into battery (kW)")
    ax.set_title("Charging curves 10 -> 80 %, 150 kW charger")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file


def plot_charging_studies(ambient_df, power_df, out_file: Path) -> Path:
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    ax = axes[0]
    temps = sorted(ambient_df["ambient_c"].unique())
    x = np.arange(len(temps))
    for k, (label, color) in enumerate((("parked", "tab:blue"), ("preconditioned", "tab:orange"))):
        d = ambient_df[ambient_df["battery_start"] == label].set_index("ambient_c").loc[temps]
        bars = ax.bar(x + (k - 0.5) * 0.4, d["time_min"], 0.4, color=color,
                      label=f"battery {label}")  # fmt: skip
        ax.bar_label(bars, fmt="%.0f", fontsize=8, padding=2)
    ax.set_xticks(x, [f"{t} degC" for t in temps])
    ax.set_ylabel("Time 10 -> 80 % (min)")
    ax.set_title("Charge time vs ambient temperature (150 kW)")
    ax.legend(fontsize=8)
    ax.grid(True, axis="y", alpha=0.3)

    ax = axes[1]
    ax.plot(power_df["charger_kw"], power_df["time_min"], "o-", lw=2)
    ax.set_xlabel("Charging station power (kW)")
    ax.set_ylabel("Time 10 -> 80 % (min)")
    ax.set_title("Charge time vs charger power (25 degC)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file
