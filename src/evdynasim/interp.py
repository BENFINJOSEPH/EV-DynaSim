"""Lookup tables with linear (1-D) and bilinear (2-D) interpolation.

Written by hand instead of using SciPy so that the C++ engine can implement
exactly the same algorithm (same index search, same clamping). That is what
makes a 1e-9 back-to-back comparison between Python and C++ possible.

Outside the grid the value is clamped to the nearest edge (no extrapolation).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike, NDArray


def _check_axis(name: str, axis: NDArray[np.float64]) -> None:
    if axis.ndim != 1 or len(axis) < 2:
        raise ValueError(f"{name} needs at least two points")
    if np.any(np.diff(axis) <= 0):
        raise ValueError(f"{name} must be strictly increasing")


def _locate(axis: NDArray[np.float64], x: NDArray[np.float64]):
    """Index i of the cell [axis[i], axis[i+1]] and weight w in [0, 1]."""
    xc = np.clip(x, axis[0], axis[-1])
    i = np.clip(np.searchsorted(axis, xc, side="right") - 1, 0, len(axis) - 2)
    w = (xc - axis[i]) / (axis[i + 1] - axis[i])
    return i, w


@dataclass(frozen=True)
class Table1D:
    x: NDArray[np.float64]
    y: NDArray[np.float64]

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", np.asarray(self.x, dtype=float))
        object.__setattr__(self, "y", np.asarray(self.y, dtype=float))
        _check_axis("Table1D x", self.x)
        if self.y.shape != self.x.shape:
            raise ValueError("Table1D x and y must have the same length")

    def __call__(self, x: ArrayLike) -> NDArray[np.float64]:
        i, w = _locate(self.x, np.asarray(x, dtype=float))
        return (1.0 - w) * self.y[i] + w * self.y[i + 1]


@dataclass(frozen=True)
class Table2D:
    """z[i, j] = f(x[i], y[j])."""

    x: NDArray[np.float64]
    y: NDArray[np.float64]
    z: NDArray[np.float64]

    def __post_init__(self) -> None:
        for name in ("x", "y", "z"):
            object.__setattr__(self, name, np.asarray(getattr(self, name), dtype=float))
        _check_axis("Table2D x", self.x)
        _check_axis("Table2D y", self.y)
        if self.z.shape != (len(self.x), len(self.y)):
            raise ValueError("Table2D z must have shape (len(x), len(y))")

    def __call__(self, x: ArrayLike, y: ArrayLike) -> NDArray[np.float64]:
        i, wx = _locate(self.x, np.asarray(x, dtype=float))
        j, wy = _locate(self.y, np.asarray(y, dtype=float))
        z = self.z
        return (
            (1 - wx) * (1 - wy) * z[i, j]
            + wx * (1 - wy) * z[i + 1, j]
            + (1 - wx) * wy * z[i, j + 1]
            + wx * wy * z[i + 1, j + 1]
        )

    @classmethod
    def from_csv(cls, path: str | Path) -> Table2D:
        """CSV grid: first row = y axis (after a label cell), first column = x axis.

        Example (x = speed in rpm, y = torque in Nm, values = loss in W):
            speed_rpm/torque_nm,0,50,100
            0,0,10,40
            1000,30,45,80
        """
        data = np.loadtxt(path, delimiter=",", skiprows=1)
        with Path(path).open(encoding="utf-8") as fh:
            header = fh.readline().strip().split(",")
        y = np.array([float(v) for v in header[1:]])
        return cls(x=data[:, 0], y=y, z=data[:, 1:])
