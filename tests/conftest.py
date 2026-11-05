"""Shared test fixtures."""

from pathlib import Path

import pytest

from evdynasim.params import load_params

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "configs"


@pytest.fixture
def simple():
    """Hand-calculation model: constant efficiencies, ideal battery (docs/03)."""
    return load_params(CONFIG_DIR / "simple_ev.yaml")


@pytest.fixture
def ev():
    """Detailed model: loss map, gearbox, ECM battery, thermal, HVAC."""
    return load_params(CONFIG_DIR / "generic_ev.yaml")
