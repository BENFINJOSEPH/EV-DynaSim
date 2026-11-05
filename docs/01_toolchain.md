# 1. Toolchain: languages, tools, libraries and setup

## 1.1 Languages and their jobs

| Language | Job in EV-DynaSim | Why this language |
|---|---|---|
| Python 3.13 | Reference model, forward simulation, data import, calibration, validation, studies, plots, test orchestration | Fast to write and to check physics. NumPy, pandas, SciPy and Matplotlib are the standard tools for engineering data |
| C++17 | Simulation engine (same backward model as Python), command line tool | Speed (about 100x faster here), strong typing, used in production simulation and embedded software. C++17 is supported by every current compiler |
| C (interface only) | `extern "C"` functions in `evdynasim_c.h` | C has a stable binary interface (ABI), C++ does not. Through the C interface the engine can be called from Python, MATLAB, LabVIEW, C# or an FMU wrapper |
| YAML | Vehicle configurations, signal mappings | Readable, supports comments, a new vehicle needs no code change |
| CMake | C++ build description | Same build on Windows (MSVC) and Linux (GCC/Clang) |

**Why Python first, then C++.** "Is the physics right?" and "is the C++
right?" are different questions. The Python model is the reference (the
*oracle*): the C++ engine must reproduce it to 1e-9 relative error on every
time step (`tests/test_cpp_vs_python.py`). This is called back-to-back testing
and is standard in model-based development.

## 1.2 Tools

| Tool | What it is | Why we use it |
|---|---|---|
| VS Code | Editor and debugger | Python and C++ in one place, integrated terminal |
| Python 3.13 (python.org) | Interpreter | Use the python.org installer, not the Microsoft Store version |
| venv | Isolated Python environment | Packages of this project do not affect other projects |
| pip | Python package installer | Installs the libraries listed in `pyproject.toml` |
| Visual Studio 2022 Build Tools | MSVC C++ compiler, linker, Windows SDK | Standard compiler on Windows |
| CMake (>= 3.20) | Build-system generator | Comes with Visual Studio; generates the MSVC project |
| CTest | Test runner of CMake | Runs the GoogleTest executable |
| Git | Version control | History of every change, tags for releases |
| ruff | Linter + formatter for Python | Finds bugs and style issues, formats consistently |
| pytest, pytest-cov | Python test framework and coverage | Tests, fixtures, parametrisation, coverage report |
| GitHub Actions (optional) | CI | The workflow in `.github/workflows/ci.yml` runs automatically if the project is ever hosted on GitHub. Not needed locally |

## 1.3 Python libraries

| Library | Used for | Where |
|---|---|---|
| NumPy | Arrays, vectorised maths | everywhere |
| pandas | Tables: CSV import, time series results, metrics | `measured.py`, `simulate.py`, `calibration.py` |
| SciPy | `optimize.least_squares` for calibration | `calibration.py` |
| Matplotlib | Figures | `plotting.py`, `report.py` |
| PyYAML | Reading configuration files | `params.py`, `measured.py` |
| ctypes (standard library) | Calling the C++ shared library | `cpp_engine.py` |
| argparse (standard library) | Command line tool | `cli.py` |

## 1.4 C++ libraries

| Library | Used for | How it is obtained |
|---|---|---|
| C++17 standard library | containers, `<cmath>`, `std::optional`, file I/O, `<chrono>` | with the compiler |
| GoogleTest 1.15 | unit tests | downloaded automatically by CMake `FetchContent` at configure time |

Deliberately **not** used: Eigen (no matrix maths needed in the backward model;
would be the first choice for an EKF), yaml-cpp (the configuration format is
small enough for a 100-line reader), pybind11 (replaced by the C interface +
ctypes, see `docs/07_cpp_engine.md`). Fewer dependencies = the engine builds
with only a compiler and CMake.

## 1.5 Setup on Windows

### Step 1 - Install

1. **Python 3.13** from python.org (64-bit). Tick *Add python.exe to PATH*.
2. **Visual Studio 2022 Build Tools** (or Visual Studio Community), workload
   *Desktop development with C++*. This includes MSVC, the Windows SDK and CMake.
3. **Git** from git-scm.com, then once:
   ```powershell
   git config --global user.name  "Benfin Joseph"
   git config --global user.email "benfinjoseph62@gmail.com"
   git config --global init.defaultBranch main
   ```
4. **VS Code**, open the `EV-DynaSim` folder and accept the recommended
   extensions (Python, Pylance, Ruff, C/C++, CMake Tools).

### Step 2 - Python environment

In the VS Code terminal (`` Ctrl + ` ``) inside `EV-DynaSim`:

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
```

`-e` = editable install (source changes take effect without reinstalling),
`[dev]` = also install pytest and ruff. Then `Ctrl + Shift + P` ->
*Python: Select Interpreter* -> `.venv`.

Optional activation so that `python` and `evdynasim` work directly:
```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned   # once
.\.venv\Scripts\Activate.ps1
```

### Step 3 - Build the C++ engine

Open **Developer PowerShell for VS 2022** (start menu), `cd` to the project:

```powershell
cmake -S cpp -B build                         # configure (downloads GoogleTest once)
cmake --build build --config Release          # compile
ctest --test-dir build -C Release --output-on-failure
build\bin\Release\evdynasim_cli.exe --config configs/generic_ev.yaml --cycle nedc
```

Output files: `build\bin\Release\evdynasim_c.dll` (C interface, used by
Python), `evdynasim_cli.exe`, `evdynasim_tests.exe`.

No internet during configure? Install GoogleTest with vcpkg and use
`-DEVDS_USE_SYSTEM_GTEST=ON`, or build without tests: `-DEVDS_BUILD_TESTS=OFF`.

### Step 4 - Check everything

```powershell
.\.venv\Scripts\python.exe -m ruff check .
.\.venv\Scripts\python.exe -m pytest          # expect: 144 passed
.\.venv\Scripts\evdynasim.exe all --config configs/generic_ev.yaml --out results
```

Or run the whole sequence with `.\scripts\build_and_test.ps1`. If the C++
library is not built, the 10 C++-related Python tests are skipped (not failed).

### Step 5 - Local version control

```powershell
git init
git add .
git commit -m "EV-DynaSim 1.0: Python + C++ EV energy simulation platform"
git tag v1.0
```

`build/`, `results/` and `data/raw/` are excluded by `.gitignore`.

## 1.6 Typical problems

| Symptom | Cause / fix |
|---|---|
| `py` not found | Python not on PATH; reinstall with the PATH option or use the full path |
| `cmake` not found | Use *Developer PowerShell for VS 2022*, not the normal PowerShell |
| GoogleTest download fails | Proxy/firewall; use `-DEVDS_BUILD_TESTS=OFF` or a local GoogleTest |
| Python says "C++ engine not found" | Build in `build/` (not another folder) or set `EVDYNASIM_CPP_LIB` to the full path of `evdynasim_c.dll` |
| Activate.ps1 blocked | `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` |
