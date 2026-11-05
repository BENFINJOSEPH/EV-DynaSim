"""Driving cycles: speed (and optional grade) as a function of time.

A ``DriveCycle`` is the input of the backward-facing simulation. Cycles come
from two sources:

* built-in NEDC, generated from its published piecewise-linear definition
  (ECE-15 urban part x4 + EUDC extra-urban part, 1180 s). No download needed.
* CSV files with columns ``time_s, speed_kmh`` and optionally ``grade_pct``.
  This is how WLTC or measured drives (e.g. the TUM Taycan data) come in.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from numpy.typing import NDArray

from evdynasim.params import KMH_TO_MPS

# Source: UNECE Regulation No. 83, Annex 4 (operating-sequence tables).
# Published distances: ECE-15 1.013 km, EUDC 6.955 km, NEDC about 11.007 km.

# ECE-15 elementary urban cycle, 195 s: (time_s, speed_kmh) breakpoints.
# Gear-change phases are modelled as constant speed, as in the usual tabulation.
_ECE15 = [
    (0, 0), (11, 0), (15, 15), (23, 15), (25, 10), (28, 0), (49, 0),
    (54, 15), (56, 15), (61, 32), (85, 32), (93, 10), (96, 0), (117, 0),
    (122, 15), (124, 15), (133, 35), (135, 35), (143, 50), (155, 50),
    (163, 35), (176, 35), (178, 32), (185, 10), (188, 0), (195, 0),
]  # fmt: skip

# EUDC extra-urban driving cycle, 400 s, max 120 km/h.
_EUDC = [
    (0, 0), (20, 0), (25, 15), (27, 15), (36, 35), (38, 35), (46, 50),
    (48, 50), (61, 70), (111, 70), (119, 50), (188, 50), (201, 70),
    (251, 70), (286, 100), (316, 100), (336, 120), (346, 120), (362, 80),
    (370, 50), (380, 0), (400, 0),
]  # fmt: skip


@dataclass(frozen=True)
class DriveCycle:
    """Time series input for one simulation run (SI units)."""

    name: str
    time_s: NDArray[np.float64]
    speed_mps: NDArray[np.float64]
    grade_pct: NDArray[np.float64]

    def __post_init__(self) -> None:
        n = len(self.time_s)
        if n < 2:
            raise ValueError("A drive cycle needs at least two samples.")
        if len(self.speed_mps) != n or len(self.grade_pct) != n:
            raise ValueError("time, speed and grade arrays must have the same length.")
        if np.any(np.diff(self.time_s) <= 0):
            raise ValueError("time_s must be strictly increasing.")
        if np.any(self.speed_mps < 0):
            raise ValueError("Speed must not be negative.")
        if not (np.all(np.isfinite(self.speed_mps)) and np.all(np.isfinite(self.grade_pct))):
            raise ValueError("Cycle contains NaN or infinite values.")

    @property
    def duration_s(self) -> float:
        return float(self.time_s[-1] - self.time_s[0])

    @property
    def distance_m(self) -> float:
        """Distance by trapezoidal integration of speed."""
        return float(np.trapezoid(self.speed_mps, self.time_s))


def _from_breakpoints(points: list[tuple[float, float]], dt: float) -> tuple[np.ndarray, ...]:
    t_bp, v_bp = np.array(points, dtype=float).T
    t = np.arange(t_bp[0], t_bp[-1] + 0.5 * dt, dt)
    return t, np.interp(t, t_bp, v_bp)


def nedc(dt: float = 1.0) -> DriveCycle:
    """New European Driving Cycle (4 x ECE-15 + EUDC), 1180 s."""
    pts: list[tuple[float, float]] = []
    offset = 0.0
    for segment in [_ECE15] * 4 + [_EUDC]:
        start = 1 if pts else 0  # do not duplicate the shared zero point
        pts.extend((t + offset, v) for t, v in segment[start:])
        offset += segment[-1][0]
    t, v_kmh = _from_breakpoints(pts, dt)
    return DriveCycle("NEDC", t, v_kmh * KMH_TO_MPS, np.zeros_like(t))


def constant_speed(speed_kmh: float, duration_s: float, dt: float = 1.0) -> DriveCycle:
    """Constant speed cycle. Useful for tests and steady-state checks."""
    t = np.arange(0.0, duration_s + 0.5 * dt, dt)
    v = np.full_like(t, speed_kmh * KMH_TO_MPS)
    return DriveCycle(f"constant {speed_kmh:g} km/h", t, v, np.zeros_like(t))


def load_csv(path: str | Path) -> DriveCycle:
    """Load a cycle from CSV with columns time_s, speed_kmh[, grade_pct]."""
    path = Path(path)
    df = pd.read_csv(path)
    missing = {"time_s", "speed_kmh"} - set(df.columns)
    if missing:
        raise ValueError(f"{path.name}: missing column(s) {sorted(missing)}")
    grade = df["grade_pct"] if "grade_pct" in df.columns else np.zeros(len(df))
    return DriveCycle(
        name=path.stem,
        time_s=df["time_s"].to_numpy(dtype=float),
        speed_mps=df["speed_kmh"].to_numpy(dtype=float) * KMH_TO_MPS,
        grade_pct=np.asarray(grade, dtype=float),
    )


BUILTIN_CYCLES = {"nedc": nedc}


def get_cycle(name_or_path: str) -> DriveCycle:
    """Return a built-in cycle by name, or load it from a CSV path."""
    key = name_or_path.lower()
    if key in BUILTIN_CYCLES:
        return BUILTIN_CYCLES[key]()
    return load_csv(name_or_path)
