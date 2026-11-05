"""Synthetic "measured" drives for developing and testing the validation pipeline.

IMPORTANT: data from this module is NOT measurement data. It is produced by
simulating a hidden "truth" vehicle whose parameters differ from the model
configuration, plus sensor noise. It lets the calibration and validation
code be developed and tested with a known answer. Real data (for example the
TUM Porsche Taycan dataset) enters through ``measured.load_measured_csv``.

The truth vehicle differs from the model in two ways:
* parameters the calibration is allowed to estimate (Crr, Cd, auxiliary
  power, HVAC coefficient) - the calibration should recover these
* structural differences it cannot estimate (more rotating inertia, 8 %
  higher motor losses) - these remain as model error after calibration,
  like in real projects
"""

from __future__ import annotations

import dataclasses

import numpy as np

from evdynasim.cycles import DriveCycle
from evdynasim.interp import Table2D
from evdynasim.measured import MeasuredDrive, interval_to_sample_power
from evdynasim.params import ModelParams, with_changes
from evdynasim.simulate import simulate_cycle

#: Parameters the calibration estimates (true values of the hidden vehicle).
TRUTH_PARAMS = {
    "body.rolling_coefficient": 0.0115,
    "body.drag_coefficient": 0.265,
    "aux.base_power_w": 650.0,
    "aux.hvac_w_per_k": 100.0,
}
#: Structural differences the calibration does not estimate.
STRUCTURAL_CHANGES = {"body.rotating_mass_factor": 1.06}
MOTOR_LOSS_FACTOR = 1.08


def truth_vehicle(p: ModelParams, structural: bool = True) -> ModelParams:
    truth = with_changes(p, TRUTH_PARAMS)
    if not structural:
        return truth
    truth = with_changes(truth, STRUCTURAL_CHANGES)
    lm = p.motor.loss_map
    if lm is not None:
        scaled = Table2D(lm.x, lm.y, lm.z * MOTOR_LOSS_FACTOR)
        truth = dataclasses.replace(truth, motor=dataclasses.replace(truth.motor, loss_map=scaled))
    return truth


def random_drive(seed: int, duration_s: float = 1500.0) -> DriveCycle:
    """Random sequence of urban, rural and motorway micro-trips with a hilly road."""
    rng = np.random.default_rng(seed)
    v_kmh: list[float] = [0.0] * 10
    targets = {"urban": (25, 50), "rural": (60, 95), "motorway": (100, 130)}
    while len(v_kmh) < duration_s:
        kind = rng.choice(list(targets), p=[0.5, 0.3, 0.2])
        target = rng.uniform(*targets[kind])
        accel = rng.uniform(0.8, 2.0) * 3.6  # km/h per s
        decel = rng.uniform(0.8, 2.5) * 3.6
        v = v_kmh[-1]
        while v != target:  # ramp up or down to the new target speed
            v = min(target, v + accel) if v < target else max(target, v - decel)
            v_kmh.append(v)
        cruise = int(rng.uniform(20, 120))
        phase = rng.uniform(0, 2 * np.pi)
        v_kmh += list(
            target + 3.0 * (np.sin(np.arange(1, cruise + 1) / 15.0 + phase) - np.sin(phase))
        )
        if kind == "urban" or rng.random() < 0.3:  # stop
            v = v_kmh[-1]
            while v > 0:
                v = max(0.0, v - decel)
                v_kmh.append(v)
            v_kmh += [0.0] * int(rng.uniform(5, 40))
    v_kmh = v_kmh[: int(duration_s)]
    while v_kmh[-1] > 0:  # finish at standstill
        v_kmh.append(max(0.0, v_kmh[-1] - 2.5 * 3.6))
    v_kmh += [0.0] * 5

    t = np.arange(float(len(v_kmh)))
    freqs = rng.uniform(1 / 900, 1 / 150, 3)
    amps = rng.uniform(0.5, 2.0, 3)
    phases = rng.uniform(0, 2 * np.pi, 3)
    grade = sum(
        a * np.sin(2 * np.pi * f * t + ph) for a, f, ph in zip(amps, freqs, phases, strict=True)
    )
    return DriveCycle(f"drive{seed:02d}", t, np.array(v_kmh) / 3.6, grade)


def synthesize_measurement(
    p_model: ModelParams,
    seed: int,
    ambient_temp_c: float,
    power_noise_w: float = 300.0,
    speed_noise_mps: float = 0.03,
    structural: bool = True,
    duration_s: float = 1500.0,
) -> MeasuredDrive:
    """Simulate the truth vehicle on a random drive and add sensor noise."""
    rng = np.random.default_rng(1000 + seed)
    cycle = random_drive(seed, duration_s)
    truth = with_changes(
        truth_vehicle(p_model, structural),
        {"env.ambient_temp_c": ambient_temp_c, "thermal.initial_temp_c": ambient_temp_c},
    )
    res = simulate_cycle(truth, cycle)
    p_samples = interval_to_sample_power(res.battery_power_w)
    n = len(cycle.time_s)
    speed_meas = np.clip(cycle.speed_mps + rng.normal(0, speed_noise_mps, n), 0, None)
    return MeasuredDrive(
        name=f"synthetic_{cycle.name}",
        time_s=cycle.time_s,
        speed_mps=speed_meas,
        grade_pct=cycle.grade_pct,
        battery_power_w=p_samples + rng.normal(0, power_noise_w, n),
        soc=res.soc + rng.normal(0, 0.002, n),
        ambient_temp_c=ambient_temp_c,
    )


def synthetic_dataset(
    p_model: ModelParams, n_drives: int = 8, seed0: int = 1, structural: bool = True
) -> list[MeasuredDrive]:
    """Drives with ambient temperatures spread from -5 to 30 degC."""
    temps = np.linspace(-5.0, 30.0, n_drives)
    order = np.random.default_rng(seed0).permutation(n_drives)
    return [
        synthesize_measurement(p_model, seed0 + i, float(temps[order[i]]), structural=structural)
        for i in range(n_drives)
    ]
