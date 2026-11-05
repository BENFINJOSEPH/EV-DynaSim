# 7. C++ engine

## 7.1 Purpose

The C++ engine implements the backward model (vehicle, powertrain, battery,
thermal) with the same equations and the same discretisation as the Python
reference. It is about 100 times faster and can be embedded in other tools
through its C interface.

## 7.2 Structure

```
cpp/
├── CMakeLists.txt
├── include/evdynasim/
│   ├── config.hpp        reader for the YAML subset (sections, numbers, strings, lists)
│   ├── interp.hpp        Table1D, Table2D (same algorithm as interp.py)
│   ├── params.hpp        struct per component + ModelParams, load_params(), validate()
│   ├── physics.hpp       road-load forces
│   ├── powertrain.hpp    PowertrainPoint compute_powertrain(...)
│   ├── battery.hpp       BatteryState, battery_step(), current_for_power()
│   ├── drive_cycle.hpp   DriveCycle, nedc(), load_cycle_csv()
│   ├── simulator.hpp     SimulationResult simulate(params, cycle), Summary
│   └── evdynasim_c.h     C interface
├── src/                  one .cpp per header + c_api.cpp
├── apps/main.cpp         evdynasim_cli
└── tests/                GoogleTest: config, physics/powertrain, battery, simulator
```

CMake targets:

| Target | Type | Content |
|---|---|---|
| `evdynasim_core` | static library | all model code |
| `evdynasim_c` | shared library (.dll/.so) | C interface, links the core |
| `evdynasim_cli` | executable | command line tool, benchmark option `--repeat N` |
| `evdynasim_tests` | executable | 25 GoogleTest tests, run with `ctest` |

Compiler warnings are on (`/W4 /permissive-` for MSVC, `-Wall -Wextra
-Wpedantic -Wshadow -Wconversion` for GCC/Clang). The code compiles without
warnings with GCC 13 and Clang with `-Werror`.

## 7.3 Design decisions

**Free functions + plain structs for stateless parts, a state struct for the
battery.** `compute_powertrain(params, force, speed)` has no state, so a free
function is the simplest correct design. The battery has state (SOC, U1, T),
held in `BatteryState` and advanced by `battery_step(params, state, ...)`.
Parameters are always passed as `const ModelParams&`: read-only, no copy.

**`std::optional<Table2D>` for the loss map.** Expresses "this vehicle may
or may not have a measured map" in the type. The code checks
`if (motor.loss_map)` and otherwise uses constant efficiencies.

**Exceptions inside, error codes at the boundary.** Invalid parameters,
unreadable files and invalid cycles throw `std::invalid_argument` /
`std::runtime_error` with a clear message. `c_api.cpp` catches everything
(`catch (const std::exception&)` and `catch (...)`) and returns `EVDS_ERROR`
plus the message in a caller-provided buffer. A C++ exception crossing a C
boundary is undefined behaviour.

**Locale-independent number parsing.** `std::stod` follows the C locale;
on a computer with German regional settings "1.5" can be read as 1. The
reader uses an `istringstream` imbued with `std::locale::classic()`. There is
a test for it.

**No external libraries in the engine.** Only the standard library. The
YAML subset reader is about 100 lines and is tested; GoogleTest is the only
dependency and only for the tests.

## 7.4 The C interface

```c
int evds_simulate(const char* config_path,
                  const double* time_s, const double* speed_mps, const double* grade_pct, size_t n,
                  double* battery_power_w,   /* n-1 values, may be NULL */
                  double* soc, double* temp_c,  /* n values, may be NULL */
                  evds_summary* summary, char* error_msg, size_t error_len);
```

Rules: only C types; the caller allocates all memory; return code + error
text instead of exceptions; `evds_summary` is a plain struct of doubles and
ints. `EVDS_API` exports the symbols on Windows (`__declspec(dllexport)`)
and sets default visibility on Linux, where all other symbols are hidden.

**Python side (`cpp_engine.py`).** `ctypes.CDLL` loads the library,
`argtypes`/`restype` declare the signature, NumPy arrays are passed as
`double*` with `arr.ctypes.data_as(...)`, and `EvdsSummary(ctypes.Structure)`
mirrors the C struct field by field.

**Why not pybind11?** pybind11 would give a nicer Python API, but the module
is tied to one Python version and needs the Python headers at build time.
The C interface is needed anyway for non-Python tools, and ctypes is part of
Python. Trade-off: the struct layout must be kept in sync manually (a test
compares every summary field, so a mismatch is detected immediately).

## 7.5 Back-to-back verification

`tests/test_cpp_vs_python.py` runs 4 cycles (NEDC, hill cycle, 130 km/h
constant, random drive with grades) x 2 configurations through both engines:

* battery power at every interval: relative difference < 1e-9
* SOC and temperature at every sample: < 1e-12
* every summary value: < 1e-9

To make this possible the C++ code reproduces the order of floating-point
operations of the Python code (for example `0.5 * rho * cd * A * (v * v)`),
and both use the same hand-written interpolation.

## 7.6 Performance

| Engine | Time per NEDC run (1180 s simulated) |
|---|---|
| Python (vectorised vehicle, battery loop) | 31 ms |
| C++ via ctypes, including reading config + loss map | 0.44 ms |
| C++ simulation only (`evdynasim_cli --repeat 200`) | 0.28 ms |

0.28 ms for 1180 simulated seconds is about 4 million times faster than real
time. The Python version is dominated by the per-step battery loop
(interpreter overhead per iteration); vectorising it is impossible because
each step depends on the previous state - the typical case where C++ pays off.
Measured on the development machine; numbers on your laptop will differ.
