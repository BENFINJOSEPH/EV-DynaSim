# 4. Project status and possible extensions

## 4.1 Milestones

| # | Milestone | Status | Main content |
|---|---|---|---|
| 1 | Steady-state model | done | road loads, power vs speed, hand-calculation tests |
| 2 | Driving-cycle simulation | done | backward simulation, regen, SOC, energy balance, NEDC, CSV cycles, CLI |
| 3 | Powertrain and battery | done | gearbox, torque envelope, loss map, 1-RC battery, thermal model, HVAC, charge limit |
| 4 | C++17 engine | done | CMake, GoogleTest, C interface, CLI, ctypes wrapper, back-to-back test, benchmark |
| 5 | Measured data, calibration, validation | done | signal mapping importer, synthetic data with known truth, least squares with confidence intervals, validation metrics, report |
| 6 | Forward simulation and control | done | ODE model, RK4 and Euler with convergence test, PI driver with feed-forward and anti-windup |
| 7 | Studies and tooling | done | sensitivity, ambient temperature, benchmark, `evdynasim all`, CI configuration |
| 8 | DC fast charging | done | BMS charge strategy (C-rate with temperature and SOC derating, charger power, CV limit), active cooling, 13 requirements with a 46-case test specification, charging studies |

Size: about 3 200 lines of Python, 1 300 lines of C++, 144 Python tests and
25 C++ tests.

## 4.2 Next steps with the highest value

1. **Real data.** Download a few drives of the TUM Porsche Taycan dataset
   (DOI 10.14459/2025mp1774217), check the licence, fill in
   `configs/datasets/tum_taycan_template.yaml`, write a Taycan configuration
   from published data (two-speed gearbox: start with the gear used most) and
   run `evdynasim validate`. This turns verification into validation and is
   the most convincing single addition for interviews.
2. **WLTC class 3** as a CSV in `data/cycles/` (from the UNECE GTR 15
   table), with a distance test like the NEDC one.
3. **EKF SOC estimator** on the simulated battery (noisy current and voltage
   as measurements, ECM as the model). Links this project to the battery
   estimation project and to the thesis topic (state estimation). Natural
   place to introduce Eigen in C++.
4. **Forward model in C++** (port `forward.py`): an RK4 integrator class and
   the PI controller as a class with state. Good C++ practice with classes
   that own state.

## 4.3 Further extensions

* Charging: preconditioning model with heater energy and timing, port the
  charging logic to C++, a cell-level model where the hottest cell sets the derating.
* MPC eco-driving or predictive regen on a short horizon, compared with the PI driver.
* Two-speed gearbox with a shift strategy (needed for the Taycan).
* OCV hysteresis, second RC pair, ageing (capacity and resistance vs cycles).
* Cabin thermal model instead of the linear HVAC term.
* FMU export (FMI 3.0 co-simulation) around the C interface.
* Parallel batch runs (the C++ engine is stateless between runs, so runs can
  be distributed over threads without locks).
* Streamlit dashboard on top of the CLI functions.
