# 5. Learning guide and interview preparation

The project is only useful for interviews if you can explain every part.
Work through it in this order. Do not move on until you can explain the
current step without looking at the code.

## 5.1 Reading order

| Step | Files | Understand | Concepts |
|---|---|---|---|
| 1 | `configs/simple_ev.yaml`, `generic_ev.yaml` | every parameter and its unit | YAML |
| 2 | `params.py` | components, validation, SI conversion, `with_changes` | frozen dataclasses, `__post_init__`, `@classmethod`, `@property` |
| 3 | `physics.py`, `tests/test_physics.py` | forces, hand calculation | NumPy vectorisation, `np.where`, pytest fixtures |
| 4 | `interp.py`, `scripts/generate_loss_map.py` | lookup tables, loss map | `np.searchsorted`, bilinear interpolation |
| 5 | `powertrain.py`, `tests/test_components.py` | motoring vs braking, regen limits | boolean masks |
| 6 | `battery.py` | ECM, current from power, discretisation | dataclasses as state |
| 7 | `simulate.py`, `tests/test_simulate.py` | full simulation, energy balance | vectorised + sequential parts |
| 8 | `cpp/include/evdynasim/*.hpp`, `cpp/src/*.cpp` | same model in C++ | structs, `const&`, `std::optional`, exceptions, namespaces |
| 9 | `cpp/CMakeLists.txt`, `cpp/tests/*.cpp` | build, targets, tests | CMake targets, GoogleTest |
| 10 | `evdynasim_c.h`, `c_api.cpp`, `cpp_engine.py` | C interface, ctypes | ABI, `extern "C"`, ownership of memory |
| 11 | `forward.py`, `tests/test_forward.py` | ODE, RK4, PI control, anti-windup | closures, numerical order |
| 12 | `measured.py`, `synthetic.py`, `calibration.py`, `report.py` | data pipeline, least squares, validation | pandas, SciPy, covariance from the Jacobian |
| 13 | `studies.py`, `cli.py` | studies, command line | argparse sub-commands |

Debugging tip: in VS Code open a test file, set a breakpoint in the function
under test, run *Debug Test*, and inspect the arrays in the Variables panel.
For C++: open the `cpp` folder with CMake Tools, select the `evdynasim_tests`
target and press F5.

## 5.2 Exercises (one git commit each)

1. Add a headwind to the drag force (air speed = v + v_wind). Python and C++,
   plus a test in both, and keep the back-to-back test green.
2. Add `range_km` (usable energy / consumption) to the summary. Which
   assumptions make this number optimistic?
3. Add WLTC class 3 as a CSV and a distance test (23.27 km, 1800 s).
4. Replace the linear HVAC term by a heat-pump COP that falls with ambient
   temperature. How does the ambient chart change?
5. Implement an EKF that estimates SOC from noisy current and terminal
   voltage on a simulated NEDC. Start with a wrong initial SOC and show
   convergence.
6. Port the forward simulation to C++ with an `Integrator` interface and
   `Euler` / `RK4` implementations.

## 5.3 Interview questions (prepare short answers)

**Physics and modelling**
1. Why does steady-state consumption have a minimum around 30 km/h? (auxiliary power divided by speed vs drag rising with v^2)
2. Why is mass the most sensitive parameter in the city and Cd on the motorway?
3. Why does consumption rise by 50 % at -15 degC in the city but much less on the motorway?
4. Why do you use a loss map instead of an efficiency map?
5. Why is battery resistance invisible in "consumption" but visible in SOC?
6. What limits regenerative braking in your model, and why does it matter for city driving?

**Numerics**
7. Why is the kinetic energy exactly zero over a stop-to-stop cycle in your backward model?
8. How did you verify that your RK4 is really fourth order?
9. Why is the RC voltage update stable for any step size, and when would the thermal update become unstable?
10. Why `2P / (E + sqrt(...))` instead of the textbook quadratic formula?

**Software**
11. Why a Python reference model and a C++ engine? How do you know they agree?
12. Why a C interface around C++? What may and may not cross it?
13. Why do exceptions not cross the C interface? What happens if they do?
14. Why did you write your own interpolation instead of using SciPy?
15. What does `std::optional<Table2D>` express for the loss map?
16. How would you run 10 000 simulations for a parameter study?

**Calibration and validation**
17. What is the difference between verification and validation?
18. Why separate calibration and validation drives?
19. In case B the calibrated Crr is outside the confidence interval of the truth. Why, and is the model still useful?
20. Cd and Crr have a correlation of -0.95. What does that mean and how would you design a test drive to separate them?
21. Why are your confidence intervals too optimistic?
22. Why did smoothing the measured speed change the predicted energy by 1 %?

**Control**
23. What does the feed-forward term do, and how large is the tracking error without it?
24. Explain integrator windup with your 25 kW example. How does conditional integration prevent it?
25. Why does the forward simulation use the backward model for energy?

## 5.4 How to present the project (5 minutes)

1. Problem: predict EV energy consumption and explain where it goes (30 s).
2. Model: forces, powertrain with loss map, battery ECM, thermal, HVAC (1 min, show the energy breakdown figure).
3. Verification: energy balance, hand calculation, RK4 order, C++ = Python (1 min).
4. Calibration and validation: synthetic truth, case A vs B, 7 % -> 0.2 % (1 min, show the validation figure).
5. Engineering insight: sensitivity and ambient temperature (1 min, show the two charts).
6. Limitations and next step: real Taycan data (30 s).
