"""Measured driving data: import, cleaning and resampling.

Every logger and dataset names its signals differently (CAN signal names,
units, sample rates). Instead of hard-coding one format, a small *mapping*
file says which column holds which quantity and how to convert it to SI.
Example: configs/datasets/csv_mapping_example.yaml.

Processing steps (in this order, each one documented in docs/06_validation.md):
1. read the CSV and rename/scale columns according to the mapping
2. drop rows with missing time or speed, sort by time, drop duplicate times
3. battery power = voltage * current if no power column exists
4. resample all signals to a uniform time base (default 1 s) by linear interpolation
5. clip small negative speeds (sensor noise at standstill) to zero
6. optionally smooth speed with a centred moving average (off by default).
   The backward model differentiates speed, so noise is amplified, but
   smoothing also lowers acceleration peaks and biases the predicted energy
   (about -1 % with a 3-sample window on the synthetic drives). Use the
   smallest window that removes visible noise and report the choice.

Comparing model and measurement
-------------------------------
The model gives the mean power over each interval [t_k, t_k+1]. A logger
samples instantaneous power at t_k. The model estimate at sample k is
therefore (P_{k-1} + P_k) / 2 (``interval_to_sample_power``), and the
comparison is made at the samples.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml
from numpy.typing import NDArray

from evdynasim.cycles import DriveCycle


@dataclass(frozen=True)
class MeasuredDrive:
    name: str
    time_s: NDArray[np.float64]
    speed_mps: NDArray[np.float64]
    grade_pct: NDArray[np.float64]
    battery_power_w: NDArray[np.float64]  # + = discharge, at the battery terminals
    soc: NDArray[np.float64]  # NaN where not measured
    ambient_temp_c: float

    def to_cycle(self) -> DriveCycle:
        return DriveCycle(self.name, self.time_s, self.speed_mps, self.grade_pct)

    @property
    def distance_km(self) -> float:
        return float(np.trapezoid(self.speed_mps, self.time_s) / 1000.0)

    @property
    def energy_kwh(self) -> float:
        """Measured battery energy (trapezoidal integration of power)."""
        return float(np.trapezoid(self.battery_power_w, self.time_s) / 3.6e6)

    def to_csv(self, path: str | Path) -> None:
        """Write in the canonical format read by ``load_measured_csv`` with the default mapping."""
        pd.DataFrame(
            {
                "time_s": self.time_s,
                "speed_kmh": self.speed_mps * 3.6,
                "grade_pct": self.grade_pct,
                "battery_power_kw": self.battery_power_w / 1e3,
                "soc_pct": self.soc * 100.0,
                "ambient_temp_c": np.full(len(self.time_s), self.ambient_temp_c),
            }
        ).to_csv(path, index=False, float_format="%.6g")


DEFAULT_MAPPING: dict[str, Any] = {
    "time": {"column": "time_s", "scale": 1.0},
    "speed_mps": {"column": "speed_kmh", "scale": 1 / 3.6},
    "grade_pct": {"column": "grade_pct", "scale": 1.0},
    "battery_power_w": {"column": "battery_power_kw", "scale": 1000.0},
    "soc": {"column": "soc_pct", "scale": 0.01},
    "ambient_temp_c": {"column": "ambient_temp_c", "scale": 1.0},
}


def load_mapping(path: str | Path | None) -> dict[str, Any]:
    if path is None:
        return DEFAULT_MAPPING
    with Path(path).open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)["signals"]


def _column(df: pd.DataFrame, spec: dict[str, Any] | None) -> NDArray[np.float64] | None:
    if not spec or not spec.get("column"):
        return None
    col = spec["column"]
    if col not in df.columns:
        raise ValueError(f"column '{col}' not found; available: {list(df.columns)[:20]}")
    return df[col].to_numpy(dtype=float) * float(spec.get("scale", 1.0)) + float(
        spec.get("offset", 0.0)
    )


def interval_to_sample_power(interval_power_w: NDArray[np.float64]) -> NDArray[np.float64]:
    """Model interval powers (N-1) -> estimated instantaneous power at the N samples."""
    p = np.asarray(interval_power_w, dtype=float)
    return np.concatenate([[p[0]], 0.5 * (p[:-1] + p[1:]), [p[-1]]])


def moving_average(x: NDArray[np.float64], window: int) -> NDArray[np.float64]:
    """Centred moving average with edge handling (window must be odd)."""
    if window <= 1:
        return x.copy()
    if window % 2 == 0:
        raise ValueError("window must be odd")
    pad = window // 2
    padded = np.pad(x, pad, mode="edge")
    return np.convolve(padded, np.ones(window) / window, mode="valid")


def load_measured_csv(
    path: str | Path,
    mapping: dict[str, Any] | None = None,
    dt_s: float = 1.0,
    smooth_window: int = 1,
    default_ambient_c: float = 20.0,
) -> MeasuredDrive:
    """Load, clean and resample one measured drive."""
    path = Path(path)
    m = mapping or DEFAULT_MAPPING
    df = pd.read_csv(path)

    t = _column(df, m.get("time"))
    v = _column(df, m.get("speed_mps"))
    if t is None or v is None:
        raise ValueError("mapping must define 'time' and 'speed_mps'")
    p = _column(df, m.get("battery_power_w"))
    if p is None:
        u, i = _column(df, m.get("battery_voltage_v")), _column(df, m.get("battery_current_a"))
        if u is None or i is None:
            raise ValueError("mapping needs 'battery_power_w' or both voltage and current")
        sign = float(m.get("current_sign", 1.0))  # -1 if the logger uses charge-positive current
        p = u * i * sign
    g = _column(df, m.get("grade_pct"))
    soc = _column(df, m.get("soc"))
    amb = _column(df, m.get("ambient_temp_c"))

    data = pd.DataFrame({"t": t, "v": v, "p": p})
    data["g"] = g if g is not None else 0.0
    data["soc"] = soc if soc is not None else np.nan
    data = data.dropna(subset=["t", "v", "p"]).sort_values("t").drop_duplicates("t")
    if len(data) < 2:
        raise ValueError(f"{path.name}: fewer than two valid samples")

    t_raw = data["t"].to_numpy() - data["t"].iloc[0]
    t_new = np.arange(0.0, t_raw[-1] + 1e-9, dt_s)

    def resample(col: str) -> NDArray[np.float64]:
        y = data[col].to_numpy(dtype=float)
        ok = np.isfinite(y)
        if not ok.any():
            return np.full(len(t_new), np.nan)
        return np.interp(t_new, t_raw[ok], y[ok])

    speed = np.clip(moving_average(resample("v"), smooth_window), 0.0, None)
    ambient = float(np.nanmean(amb)) if amb is not None else default_ambient_c
    return MeasuredDrive(
        name=path.stem,
        time_s=t_new,
        speed_mps=speed,
        grade_pct=resample("g"),
        battery_power_w=resample("p"),
        soc=resample("soc"),
        ambient_temp_c=ambient,
    )
