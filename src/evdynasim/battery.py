"""Battery pack: 1-RC equivalent circuit model with lumped thermal model.

Circuit (pack level):

    OCV(SOC) ──R0(T)──┬──R1(T)──┬── terminal (+)
                      └──C1─────┘
    U_term = OCV - U1 - R0*I            (I > 0 = discharge)
    dU1/dt = -U1/tau1 + I/C1,  tau1 = R1*C1

Thermal:
    m c dT/dt = I^2 R0 + U1^2/R1 - hA (T - T_amb)

Discretisation (one step of length dt, inputs held constant):
* current from the requested terminal power (quadratic, see below)
* U1 with the exact zero-order-hold solution (stable for any dt)
* SOC by Coulomb counting, T by explicit Euler (thermal time constant is
  hours, so Euler with dt = 1 s is far inside its stability limit)

The same step function is used by the backward and the forward simulation,
and is reproduced line by line in the C++ engine (cpp/src/battery.cpp).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from evdynasim.params import ModelParams


@dataclass
class BatteryState:
    soc: float
    u1_v: float
    temp_c: float


@dataclass
class BatteryStep:
    """What happened during one step."""

    current_a: float
    terminal_v: float
    ocv_v: float
    loss_w: float  # I^2 R0 + U1*I : electrical power not reaching the terminals
    heat_w: float  # I^2 R0 + U1^2/R1 : heat into the thermal mass
    power_limited: bool  # requested power not deliverable (above limit or beyond max power)


def initial_state(p: ModelParams) -> BatteryState:
    return BatteryState(soc=p.battery.initial_soc, u1_v=0.0, temp_c=p.thermal.initial_temp_c)


def current_for_power(e_v: float, r0: float, power_w: float) -> tuple[float, bool]:
    """Solve P = (E - R0*I) * I for I (E = OCV - U1).

    I = 2P / (E + sqrt(E^2 - 4 R0 P)) is the root with the smaller |I|.
    This form is the numerically stable version of the usual quadratic
    formula and also works for R0 = 0 (gives I = P/E).
    If E^2 < 4 R0 P the power is above the maximum the pack can deliver
    (P_max = E^2 / 4R0). Then the current at maximum power is returned and
    the step is flagged.
    """
    disc = e_v * e_v - 4.0 * r0 * power_w
    if disc < 0.0:
        return e_v / (2.0 * r0), True
    return 2.0 * power_w / (e_v + math.sqrt(disc)), False


def step(
    p: ModelParams, state: BatteryState, terminal_power_w: float, dt_s: float
) -> tuple[BatteryState, BatteryStep]:
    """Advance the battery by one step with constant terminal power."""
    bat, th = p.battery, p.thermal
    ocv = float(bat.ocv_pack_v(state.soc))
    r0 = float(bat.r0_pack_ohm(state.temp_c))
    r1 = float(bat.r1_pack_ohm(state.temp_c))

    limited = terminal_power_w > bat.max_discharge_power_w
    current, beyond_max = current_for_power(ocv - state.u1_v, r0, terminal_power_w)
    limited = limited or beyond_max

    terminal_v = ocv - state.u1_v - r0 * current
    loss = current * current * r0 + state.u1_v * current
    heat = current * current * r0 + (state.u1_v * state.u1_v / r1 if r1 > 0.0 else 0.0)

    decay = math.exp(-dt_s / bat.tau1_s)
    u1_next = state.u1_v * decay + r1 * (1.0 - decay) * current
    soc_next = state.soc - current * dt_s / (3600.0 * bat.capacity_ah)
    q_out = th.h_a_w_k * (state.temp_c - p.env.ambient_temp_c)
    temp_next = state.temp_c + dt_s * (heat - q_out) / (th.mass_kg * th.heat_capacity_j_kgk)

    return (
        BatteryState(soc=soc_next, u1_v=u1_next, temp_c=temp_next),
        BatteryStep(current, terminal_v, ocv, loss, heat, limited),
    )
