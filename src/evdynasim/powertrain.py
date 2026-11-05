"""Powertrain: wheel force/speed -> motor operating point -> electrical power.

Chain (motoring):  battery -> inverter+motor -> gearbox -> wheel
Chain (regen):     wheel -> gearbox -> motor+inverter -> battery

Design decision: loss map instead of efficiency map
---------------------------------------------------
Motor characteristics are stored as a *loss* map P_loss(speed, |torque|) in W,
not as an efficiency map. Efficiency is 0/0 at zero torque or zero speed and
interpolating it near the axes produces divisions by almost zero. Losses are
finite everywhere, interpolate smoothly, and work the same way for motoring
(P_elec = P_mech + P_loss) and for regen (P_elec = P_mech - P_loss).
Losses are assumed symmetric in torque sign (same loss for +T and -T).

All functions are vectorised over time steps. Sign convention: positive
electrical power = power drawn from the battery.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from evdynasim.params import ModelParams

RADS_TO_RPM = 60.0 / (2.0 * np.pi)


@dataclass
class PowertrainResult:
    """Per-interval power flows (all in W, arrays of equal length)."""

    motor_speed_rads: NDArray[np.float64]
    motor_torque_nm: NDArray[np.float64]  # + motoring, - regen
    motor_elec_w: NDArray[np.float64]  # + drawn from battery, - returned
    gearbox_loss_w: NDArray[np.float64]
    motor_loss_w: NDArray[np.float64]
    friction_brake_w: NDArray[np.float64]
    regen_wheel_w: NDArray[np.float64]  # braking power taken by the motor at the wheel (>= 0)
    torque_limited: NDArray[np.bool_]  # demand outside the motor envelope


def motor_loss_w(p: ModelParams, speed_rads, torque_nm, motoring) -> NDArray[np.float64]:
    """Motor + inverter loss at the given operating points."""
    m = p.motor
    speed_rads = np.asarray(speed_rads, dtype=float)
    torque_nm = np.asarray(torque_nm, dtype=float)
    if m.loss_map is not None:
        return m.loss_map(speed_rads * RADS_TO_RPM, np.abs(torque_nm))
    p_mech = np.abs(torque_nm) * speed_rads
    return np.where(
        motoring,
        p_mech * (1.0 / m.constant_efficiency - 1.0),
        p_mech * (1.0 - m.constant_regen_efficiency),
    )


def compute_powertrain(
    p: ModelParams, wheel_force_n: NDArray[np.float64], speed_mps: NDArray[np.float64]
) -> PowertrainResult:
    """Map wheel force at speed to electrical motor power for every interval."""
    r, g = p.body.wheel_radius_m, p.drivetrain.gear_ratio
    eta_gb = p.drivetrain.gearbox_efficiency
    v = np.asarray(speed_mps, dtype=float)
    f = np.asarray(wheel_force_n, dtype=float)

    w_motor = v / r * g
    p_wheel = f * v
    motoring = p_wheel > 0.0
    braking = p_wheel < 0.0
    t_env = p.motor.max_torque_at(w_motor)
    safe_w = np.where(w_motor > 0.0, w_motor, 1.0)  # avoid 0/0 where it is masked anyway

    # ---- motoring: wheel -> gearbox -> motor ------------------------------
    p_mech_drive = np.where(motoring, p_wheel / eta_gb, 0.0)
    t_drive = np.where(motoring, p_mech_drive / safe_w, 0.0)

    # ---- braking: split between regen and friction brakes -----------------
    p_brake = np.where(braking, -p_wheel, 0.0)
    regen_wheel = np.minimum(p.brakes.regen_fraction * p_brake, p.brakes.max_regen_power_w)
    regen_wheel = np.where(v >= p.brakes.min_regen_speed_mps, regen_wheel, 0.0)
    # motor torque envelope also limits regen: T = P*eta_gb/w <= T_env
    regen_wheel = np.minimum(regen_wheel, t_env * w_motor / eta_gb)
    p_mech_regen = regen_wheel * eta_gb
    t_regen = np.where(braking, p_mech_regen / safe_w, 0.0)

    # ---- motor losses -------------------------------------------------------
    loss_drive = np.where(motoring, motor_loss_w(p, w_motor, t_drive, True), 0.0)
    loss_regen = np.where(
        braking & (regen_wheel > 0), motor_loss_w(p, w_motor, t_regen, False), 0.0
    )
    # Regen only where it actually returns energy; otherwise friction brakes only.
    useless = braking & (p_mech_regen <= loss_regen)
    regen_wheel = np.where(useless, 0.0, regen_wheel)
    p_mech_regen = np.where(useless, 0.0, p_mech_regen)
    t_regen = np.where(useless, 0.0, t_regen)
    loss_regen = np.where(useless, 0.0, loss_regen)

    motor_elec = (p_mech_drive + loss_drive) - (p_mech_regen - loss_regen)
    gearbox_loss = (p_mech_drive - np.where(motoring, p_wheel, 0.0)) + (regen_wheel - p_mech_regen)

    return PowertrainResult(
        motor_speed_rads=w_motor,
        motor_torque_nm=t_drive - t_regen,
        motor_elec_w=motor_elec,
        gearbox_loss_w=gearbox_loss,
        motor_loss_w=loss_drive + loss_regen,
        friction_brake_w=p_brake - regen_wheel,
        regen_wheel_w=regen_wheel,
        torque_limited=(motoring & (t_drive > t_env * (1 + 1e-12)))
        | (w_motor > p.motor.max_speed_rads),
    )
