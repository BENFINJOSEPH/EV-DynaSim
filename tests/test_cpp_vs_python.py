"""Back-to-back test: the C++ engine must reproduce the Python reference model.

Both implementations use the same equations, the same interpolation and the
same discretisation, so results must agree to round-off (1e-9 relative).
Skipped automatically when the C++ library has not been built.
"""

import numpy as np
import pytest

from evdynasim import cpp_engine
from evdynasim.cycles import DriveCycle, constant_speed, nedc
from evdynasim.params import load_params
from evdynasim.simulate import simulate_cycle

from .conftest import CONFIG_DIR
from .test_simulate import hill_cycle

pytestmark = pytest.mark.skipif(not cpp_engine.is_available(), reason="C++ engine not built")


def random_drive(seed: int) -> DriveCycle:
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, 900.0)
    v = np.clip(np.cumsum(rng.normal(0, 0.6, t.size)), 0, 36)
    v[:5] = 0
    v[-5:] = 0
    g = np.interp(t, np.linspace(0, 899, 10), rng.uniform(-5, 5, 10))
    return DriveCycle(f"random{seed}", t, v, g)


CYCLES = [nedc, hill_cycle, lambda: constant_speed(130.0, 600.0), lambda: random_drive(1)]


@pytest.mark.parametrize("cfg", ["simple_ev.yaml", "generic_ev.yaml"])
@pytest.mark.parametrize("make_cycle", CYCLES)
def test_cpp_matches_python(cfg, make_cycle):
    path = CONFIG_DIR / cfg
    cycle = make_cycle()
    py = simulate_cycle(load_params(path), cycle)
    cpp = cpp_engine.simulate_cpp(path, cycle)

    np.testing.assert_allclose(cpp["battery_power_w"], py.battery_power_w, rtol=1e-9, atol=1e-6)
    np.testing.assert_allclose(cpp["soc"], py.soc, rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(cpp["temp_c"], py.temp_c, rtol=1e-12, atol=1e-9)
    for key, value in py.summary.items():
        assert cpp["summary"][key] == pytest.approx(value, rel=1e-9, abs=1e-9), key


def test_cpp_reports_errors_instead_of_crashing(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("vehicle:\n  mass_kg: -1\n")
    with pytest.raises(RuntimeError, match="C\\+\\+ engine error"):
        cpp_engine.simulate_cpp(bad, nedc())
