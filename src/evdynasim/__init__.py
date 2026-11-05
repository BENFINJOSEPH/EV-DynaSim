"""EV-DynaSim: electric vehicle energy simulation and validation platform.

Python package: reference model, forward simulation, calibration, validation
and studies. The C++17 engine in cpp/ implements the same backward model and
is verified against this package (tests/test_cpp_vs_python.py).
"""

from evdynasim.params import ModelParams, load_params, with_changes
from evdynasim.simulate import SimulationResult, simulate_cycle, steady_state_power_w

__all__ = [
    "ModelParams",
    "with_changes",
    "load_params",
    "SimulationResult",
    "simulate_cycle",
    "steady_state_power_w",
]

__version__ = "1.0.0"
