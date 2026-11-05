"""Verification of interpolation tables, powertrain and battery model."""

import math

import numpy as np
import pytest

from evdynasim import battery
from evdynasim.interp import Table1D, Table2D
from evdynasim.params import with_changes
from evdynasim.powertrain import compute_powertrain, motor_loss_w


# ---- interpolation -----------------------------------------------------------
def test_table1d_interpolates_and_clamps():
    t = Table1D([0.0, 1.0, 3.0], [10.0, 20.0, 0.0])
    assert t(0.5) == pytest.approx(15.0)
    assert t(2.0) == pytest.approx(10.0)
    assert t(-5.0) == pytest.approx(10.0)  # clamped
    assert t(9.0) == pytest.approx(0.0)  # clamped


def test_table2d_reproduces_bilinear_function_exactly():
    x, y = np.array([0.0, 1.0, 2.0]), np.array([0.0, 10.0])
    z = 1 + 2 * x[:, None] + 0.3 * y[None, :] + 0.05 * x[:, None] * y[None, :]
    t = Table2D(x, y, z)
    for xi, yi in [(0.3, 4.0), (1.7, 9.0), (2.0, 10.0)]:
        assert t(xi, yi) == pytest.approx(1 + 2 * xi + 0.3 * yi + 0.05 * xi * yi)


def test_table_rejects_unsorted_axis():
    with pytest.raises(ValueError):
        Table1D([0.0, 2.0, 1.0], [1.0, 2.0, 3.0])


# ---- motor and powertrain ----------------------------------------------------
def test_loss_map_is_loaded_and_zero_at_standstill(ev):
    assert ev.motor.loss_map is not None
    assert motor_loss_w(ev, 0.0, 0.0, True) == pytest.approx(0.0)


def test_peak_efficiency_is_realistic(ev):
    # 150 Nm at 500 rad/s (about 4800 rpm): expect about 96 %
    p_mech = 150 * 500
    eta = p_mech / (p_mech + motor_loss_w(ev, 500.0, 150.0, True))
    assert 0.95 < eta < 0.975


def test_torque_envelope(ev):
    m = ev.motor
    assert m.max_torque_at(100.0) == pytest.approx(310.0)  # below base speed
    assert m.max_torque_at(1000.0) == pytest.approx(150.0)  # 150 kW / 1000 rad/s


def test_powertrain_power_balance_motoring(ev):
    f, v = np.array([800.0]), np.array([20.0])
    pt = compute_powertrain(ev, f, v)
    assert pt.motor_elec_w[0] == pytest.approx(
        f[0] * v[0] + pt.gearbox_loss_w[0] + pt.motor_loss_w[0]
    )


def test_powertrain_power_balance_braking(ev):
    f, v = np.array([-2500.0]), np.array([20.0])
    pt = compute_powertrain(ev, f, v)
    brake = -f[0] * v[0]
    assert pt.regen_wheel_w[0] + pt.friction_brake_w[0] == pytest.approx(brake)
    assert -pt.motor_elec_w[0] == pytest.approx(
        pt.regen_wheel_w[0] - pt.gearbox_loss_w[0] - pt.motor_loss_w[0]
    )


def test_no_regen_when_losses_exceed_recoverable_power(ev):
    # very gentle braking at high motor speed: iron losses exceed the regen power
    pt = compute_powertrain(ev, np.array([-5.0]), np.array([35.0]))
    assert pt.regen_wheel_w[0] == 0.0
    assert pt.motor_elec_w[0] == 0.0


def test_torque_limit_flag(ev):
    pt = compute_powertrain(ev, np.array([12000.0]), np.array([10.0]))  # > 8.4 kN available
    assert pt.torque_limited[0]


# ---- battery -----------------------------------------------------------------
def test_current_for_power_ideal_battery():
    i, limited = battery.current_for_power(400.0, 0.0, 40_000.0)
    assert i == pytest.approx(100.0) and not limited


def test_current_for_power_satisfies_power_equation():
    e, r0, p = 380.0, 0.05, 50_000.0
    i, _ = battery.current_for_power(e, r0, p)
    assert (e - r0 * i) * i == pytest.approx(p)


def test_current_beyond_max_power_is_flagged():
    e, r0 = 380.0, 0.05  # P_max = E^2 / 4R0 = 722 kW
    _, limited = battery.current_for_power(e, r0, 800_000.0)
    assert limited


def test_pack_values(ev):
    b = ev.battery
    assert b.capacity_ah == 200.0
    assert b.r0_pack_ohm(25.0) == pytest.approx(0.001 * 96 / 4)
    assert 68.0 < b.nominal_energy_kwh < 75.0


def test_resistance_rises_in_the_cold(ev):
    factor = ev.battery.resistance_factor(-10.0)
    assert factor == pytest.approx(math.exp(3000 * (1 / 263.15 - 1 / 298.15)))
    assert 3.5 < factor < 4.0


def test_charge_limit_derates_near_full(ev):
    b = ev.battery
    assert b.charge_limit_w(0.5) == b.max_charge_power_w
    assert b.charge_limit_w(0.95) == pytest.approx(0.5 * b.max_charge_power_w)
    assert b.charge_limit_w(1.0) == 0.0


def test_coulomb_counting(ev):
    state = battery.initial_state(ev)
    p = 0.0
    for _ in range(3600):  # one hour at a power that gives roughly 20 A
        state, info = battery.step(ev, state, 7600.0, 1.0)
        p += info.current_a
    charge_ah = p / 3600.0
    assert ev.battery.initial_soc - state.soc == pytest.approx(charge_ah / 200.0)


def test_rc_voltage_converges_to_i_r1(ev):
    isothermal = with_changes(ev, {"thermal.mass_kg": 1e9})  # temperature stays constant
    state = battery.initial_state(isothermal)
    for _ in range(600):  # 20 time constants
        state, info = battery.step(isothermal, state, 20_000.0, 1.0)
    r1 = isothermal.battery.r1_pack_ohm(25.0)
    assert state.u1_v == pytest.approx(info.current_a * r1, rel=1e-3)


def test_battery_heats_up_under_load(ev):
    state = battery.initial_state(ev)
    for _ in range(600):
        state, _ = battery.step(ev, state, 60_000.0, 1.0)
    assert state.temp_c > ev.thermal.initial_temp_c + 0.1
