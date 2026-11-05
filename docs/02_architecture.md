# 2. Architecture and design decisions

Each decision is written as *decision - reason - alternative*. That is the
format to use when an interviewer asks "why did you do it this way?".

## 2.1 Components and data flow

```
                       configs/generic_ev.yaml  +  configs/maps/generic_pmsm_loss.csv
                                     │
                         params.py (Python) / params.cpp (C++)
                                     │  ModelParams: Body, Environment, Drivetrain,
                                     │  Motor, Brakes, Auxiliaries, Battery, Thermal
                                     ▼
 cycles.py (NEDC, CSV) ──► DriveCycle ──► simulate.py ──────────────► SimulationResult
 measured.py (logger CSV) ─► MeasuredDrive ─┘   │ per interval:            │ time series,
 synthetic.py (known truth) ─┘                  │  physics.py   forces     │ energy summary,
                                                │  powertrain.py motor     │ energy balance
 forward.py: PI driver + RK4 ─► achieved ───────┘  battery.py   ECM+thermal▼
             trajectory                                           plotting.py, report.py
                                                                  studies.py, calibration.py

 C++: cpp/src/*.cpp implements params, physics, powertrain, battery, drive_cycle,
      simulator ─► evdynasim_c (shared library, C interface) ─► cpp_engine.py (ctypes)
                └─► evdynasim_cli (executable)
```

## 2.2 Modelling decisions

**D1 - Backward simulation as the core, forward simulation on top.**
The backward model (speed in, power out) needs no ODE solver, is exact for
its discretisation, and is what consumption studies and certification cycles
use. The forward model (force in, speed out) is needed to check whether the
car can follow the cycle and to design controllers. Design choice: the
forward simulation produces a *trajectory*, which is then passed through the
same backward energy model. One energy model, two ways to get the motion,
no duplicated powertrain code.

**D2 - Interval-based discretisation.** N samples give N-1 intervals with
a = dv/dt and the mean speed. Then the inertial work equals the exact change
of kinetic energy, so the energy balance closes to machine precision. With
central differences on the samples it would not, and a small numerical
residual would hide real bugs.

**D3 - Loss map instead of efficiency map.** Efficiency is 0/0 at zero
torque or speed; interpolating it near the axes divides by numbers close to
zero. Losses are finite everywhere, interpolate smoothly and work for both
directions: P_elec = P_mech + P_loss (motoring), P_mech - P_loss (regen).

**D4 - Regen limited at four points.** Brake blending fraction, wheel power
limit, motor torque envelope, and battery charge limit (which depends on SOC
and therefore lives in the sequential battery loop). Regen is switched off
where the motor losses would exceed the recovered power. Everything not
recovered goes to the friction brakes, so the energy balance still closes.

**D5 - Battery: 1-RC equivalent circuit with exact discretisation.** The
RC voltage uses the zero-order-hold solution exp(-dt/tau), which is stable
for any step size. Thermal state uses explicit Euler, stable while
dt < 2 m c / hA (here 30 000 s, far above 1 s). A test that accidentally
violated this limit (hA = 1e6 W/K) diverged, which is why the limit is now
documented.

**D6 - Current from power with the stable quadratic formula.**
I = 2P / (E + sqrt(E^2 - 4 R0 P)) instead of (E - sqrt(...)) / 2R0. Same
root, no cancellation error, and it also works for R0 = 0.

**D7 - Consumption measured at the battery terminals.** Like DC energy
measurements on a test bench. Battery internal losses are reported
separately (they are part of the chemical energy, not the terminal energy).
This is why battery resistance has no effect on "consumption" in the
sensitivity study, but it does change SOC and temperature.

## 2.3 Software decisions

**D8 - One immutable, validated parameter object per component.**
`@dataclass(frozen=True)` in Python, `struct` + `validate()` in C++. Invalid
values fail immediately with a clear message. Variants are new objects:
`with_changes(p, {"body.mass_kg": 2100})` - used by studies and calibration.

**D9 - SI units inside, engineering units only at the boundaries.** The YAML
uses kW, km/h, rpm, mOhm; conversion happens once while loading. Temperatures
stay in degC and are converted to K only in the Arrhenius term.

**D10 - Vectorised where stateless, sequential where stateful.** Vehicle and
powertrain have no state: NumPy over all intervals. Battery has state (SOC,
U1, T) and the charge limit depends on it: explicit loop. Mixing both keeps
the Python model fast enough (30 ms per NEDC) and easy to port line by line.

**D11 - Hand-written interpolation.** Not SciPy, so that Python and C++ run
exactly the same algorithm (index search, clamping, weights). This is what
makes a 1e-9 comparison possible.

**D12 - C++ engine with a C interface, called from Python with ctypes.**
See `docs/07_cpp_engine.md`. One shared library serves Python, MATLAB and
anything else; no Python headers or pybind11 needed at build time.

**D13 - Signal mapping for measured data.** Every logger names and scales
signals differently. A YAML mapping (column, scale, offset, current sign)
keeps the importer generic; supporting a new dataset means writing a mapping
file, not code.

**D14 - Synthetic data with a known truth for developing the validation.**
Real data has no known answer. With synthetic drives the calibration can be
*verified* (does it recover the true parameters?) before it is used on real
data, and the effect of structural model errors can be studied on purpose.

**D15 - Command line tool and reproducible outputs.** Every figure in the
docs is produced by `evdynasim all`. Results are not committed; they are
regenerated.

**D16 - Raw data never committed.** `data/raw/` is ignored. Datasets are
large and have their own licences.

## 2.4 Testing strategy

| Level | Example | Where |
|---|---|---|
| Hand calculations | drag at 100 km/h = 304.36 N, battery power 15 344 W | `tests/test_physics.py`, `cpp/tests/test_physics.cpp` |
| Physical properties | drag x4 when speed x2; grade force antisymmetric | `test_physics.py` |
| Component balances | P_elec = P_wheel + gearbox loss + motor loss (motoring and braking) | `test_components.py`, `test_physics.cpp` |
| Battery identities | OCV * I = P_terminal + losses every step; Coulomb counting; U1 -> I R1 | `test_components.py`, `test_battery.cpp` |
| Conservation | energy balance closes to 1e-10 on NEDC, hill cycle, with regen curtailment | `test_simulate.py`, `test_simulator.cpp` |
| Regression | simple config reproduces the v0.2 numbers | both |
| Numerical methods | Euler order 1, RK4 order 4 vs analytical coast-down | `test_forward.py` |
| Control | tracking error, energy forward vs backward < 1 %, anti-windup | `test_forward.py` |
| Back-to-back | C++ = Python to 1e-9 on 4 cycles x 2 configs | `test_cpp_vs_python.py` |
| Data pipeline | import with unsorted/duplicate/missing rows, mapping files, CSV round trip | `test_validation.py` |
| Calibration verification | true parameters recovered with matched model structure | `test_validation.py` |
| End-to-end | CLI commands write all outputs | `test_cli.py` |
| Error handling | invalid parameters, invalid cycles, C++ errors returned through the C interface | several |

Verification = "did I build the model right?" (everything above).
Validation = "did I build the right model?" (needs measured data, `docs/06_validation.md`).
