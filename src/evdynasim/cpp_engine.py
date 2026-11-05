"""Python access to the C++ engine through its C interface (ctypes).

Design decision: ctypes + C ABI instead of pybind11
---------------------------------------------------
pybind11 is the usual way to expose C++ to Python, but it ties the compiled
module to one Python version and needs the Python headers at build time.
The engine already has a C interface (needed anyway for MATLAB, LabVIEW or an
FMU wrapper), and ctypes ships with Python. So the same evdynasim_c.dll is
used from C, from Python and from any other tool, with no extra dependency.

The library is searched in:
1. the path in the environment variable EVDYNASIM_CPP_LIB
2. <repo>/build/bin/ and <repo>/build/bin/Release/ (default CMake output)
"""

from __future__ import annotations

import ctypes
import os
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np

from evdynasim.cycles import DriveCycle

REPO_DIR = Path(__file__).resolve().parents[2]

_SUMMARY_FIELDS = [
    "duration_s", "distance_km", "mean_speed_kmh", "max_speed_kmh",
    "battery_energy_kwh", "consumption_kwh_per_100km", "chemical_energy_kwh",
    "regen_energy_kwh", "regen_curtailed_kwh",
    "aero_kwh", "rolling_kwh", "grade_kwh", "kinetic_change_kwh",
    "gearbox_loss_kwh", "motor_loss_kwh", "friction_brake_kwh", "aux_kwh", "battery_loss_kwh",
    "peak_battery_power_kw", "peak_regen_power_kw", "max_current_a", "min_terminal_voltage_v",
    "initial_soc", "final_soc", "max_temp_c", "final_temp_c",
]  # fmt: skip


class EvdsSummary(ctypes.Structure):
    """Must match ``evds_summary`` in cpp/include/evdynasim/evdynasim_c.h field by field."""

    _fields_ = [(name, ctypes.c_double) for name in _SUMMARY_FIELDS] + [
        ("torque_limited_steps", ctypes.c_int),
        ("battery_limited_steps", ctypes.c_int),
    ]


def _library_name() -> str:
    if sys.platform.startswith("win"):
        return "evdynasim_c.dll"
    if sys.platform == "darwin":
        return "evdynasim_c.dylib"
    return "evdynasim_c.so"


def find_library() -> Path | None:
    env = os.environ.get("EVDYNASIM_CPP_LIB")
    if env:
        return Path(env) if Path(env).exists() else None
    for folder in ("build/bin", "build/bin/Release", "build/bin/RelWithDebInfo", "build/bin/Debug"):
        candidate = REPO_DIR / folder / _library_name()
        if candidate.exists():
            return candidate
    return None


def is_available() -> bool:
    return find_library() is not None


@lru_cache(maxsize=1)
def _load() -> ctypes.CDLL:
    path = find_library()
    if path is None:
        raise RuntimeError(
            "C++ engine not found. Build it first: cmake -S cpp -B build && "
            "cmake --build build --config Release (see docs/01_toolchain.md)"
        )
    lib = ctypes.CDLL(str(path))
    dbl_p = ctypes.POINTER(ctypes.c_double)
    lib.evds_version.restype = ctypes.c_char_p
    lib.evds_simulate.restype = ctypes.c_int
    lib.evds_simulate.argtypes = [
        ctypes.c_char_p, dbl_p, dbl_p, dbl_p, ctypes.c_size_t,
        dbl_p, dbl_p, dbl_p, ctypes.POINTER(EvdsSummary), ctypes.c_char_p, ctypes.c_size_t,
    ]  # fmt: skip
    return lib


def version() -> str:
    return _load().evds_version().decode()


def simulate_cpp(config_path: str | Path, cycle: DriveCycle) -> dict:
    """Run the C++ engine. Returns arrays and a summary dict like the Python model."""
    lib = _load()
    n = len(cycle.time_s)
    dbl_p = ctypes.POINTER(ctypes.c_double)

    def ptr(a: np.ndarray):
        return a.ctypes.data_as(dbl_p)

    t = np.ascontiguousarray(cycle.time_s, dtype=float)
    v = np.ascontiguousarray(cycle.speed_mps, dtype=float)
    g = np.ascontiguousarray(cycle.grade_pct, dtype=float)
    p_batt = np.empty(n - 1)
    soc = np.empty(n)
    temp = np.empty(n)
    summary = EvdsSummary()
    err = ctypes.create_string_buffer(512)

    rc = lib.evds_simulate(
        str(config_path).encode(), ptr(t), ptr(v), ptr(g), n,
        ptr(p_batt), ptr(soc), ptr(temp), ctypes.byref(summary), err, len(err),
    )  # fmt: skip
    if rc != 0:
        raise RuntimeError(f"C++ engine error: {err.value.decode()}")
    return {
        "battery_power_w": p_batt,
        "soc": soc,
        "temp_c": temp,
        "summary": {name: getattr(summary, name) for name, _ in EvdsSummary._fields_},
    }
