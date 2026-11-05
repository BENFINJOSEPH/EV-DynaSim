"""Parameter calibration and validation against measured drives.

Calibration = nonlinear least squares:

    min_theta  sum_drives sum_k ( P_sim,k(theta) - P_meas,k )^2

theta are uncertain parameters (default: Crr, Cd, auxiliary base power,
HVAC coefficient). Solved with scipy.optimize.least_squares (trust-region
reflective, respects bounds). Parameters are normalised by their initial
value so the optimiser sees numbers of order 1.

Uncertainty: from the Jacobian J at the optimum,
    Cov(theta) = s^2 (J^T J)^-1,   s^2 = sum r^2 / (N - n_p)
This assumes independent residuals, which is optimistic for time series
(neighbouring samples are correlated), so the confidence intervals are a
lower bound. The correlation matrix shows which parameters the data cannot
separate (for example Cd and Crr if all driving is at one speed).

Validation = run the calibrated model on drives NOT used for calibration and
report error metrics. Calibrating and evaluating on the same drives would
only measure how well the optimiser fits, not how well the model predicts.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

from evdynasim.measured import MeasuredDrive, interval_to_sample_power
from evdynasim.params import ModelParams, get_value, with_changes
from evdynasim.simulate import simulate_cycle


@dataclass(frozen=True)
class CalibParam:
    key: str  # dotted path, e.g. "body.rolling_coefficient"
    lower: float
    upper: float


DEFAULT_PARAMS = (
    CalibParam("body.rolling_coefficient", 0.005, 0.020),
    CalibParam("body.drag_coefficient", 0.18, 0.40),
    CalibParam("aux.base_power_w", 0.0, 3000.0),
    CalibParam("aux.hvac_w_per_k", 0.0, 400.0),
)


def for_drive(p: ModelParams, drive: MeasuredDrive) -> ModelParams:
    """Model parameters with the ambient conditions of one drive."""
    return with_changes(
        p,
        {
            "env.ambient_temp_c": drive.ambient_temp_c,
            "thermal.initial_temp_c": drive.ambient_temp_c,
        },
    )


def simulate_drive(p: ModelParams, drive: MeasuredDrive):
    return simulate_cycle(for_drive(p, drive), drive.to_cycle())


def sample_residual_w(res, drive: MeasuredDrive) -> np.ndarray:
    """Model minus measurement at the samples (interior samples only)."""
    return interval_to_sample_power(res.battery_power_w)[1:-1] - drive.battery_power_w[1:-1]


def metrics(res, drive: MeasuredDrive) -> dict[str, float]:
    err = sample_residual_w(res, drive)
    e_sim = res.summary["battery_energy_kwh"]
    e_meas = drive.energy_kwh
    dist_km = res.summary["distance_km"]
    return {
        "rmse_kw": float(np.sqrt(np.mean(err**2)) / 1e3),
        "mae_kw": float(np.mean(np.abs(err)) / 1e3),
        "bias_kw": float(np.mean(err) / 1e3),
        "max_abs_error_kw": float(np.max(np.abs(err)) / 1e3),
        "energy_meas_kwh": e_meas,
        "energy_sim_kwh": e_sim,
        "energy_error_pct": 100.0 * (e_sim - e_meas) / e_meas,
        "consumption_meas_kwh_100km": 100.0 * e_meas / dist_km,
        "consumption_sim_kwh_100km": 100.0 * e_sim / dist_km,
    }


def evaluate(p: ModelParams, drives: list[MeasuredDrive]) -> pd.DataFrame:
    """Error metrics per drive (one row per drive)."""
    rows = []
    for d in drives:
        res = simulate_drive(p, d)
        m = metrics(res, d)
        rows.append({"drive": d.name, "ambient_c": d.ambient_temp_c,
                     "distance_km": res.summary["distance_km"], **m})  # fmt: skip
    return pd.DataFrame(rows)


@dataclass
class CalibrationResult:
    params: ModelParams
    table: pd.DataFrame  # parameter, initial, estimate, std, ci95_low, ci95_high
    correlation: pd.DataFrame
    rmse_before_kw: float
    rmse_after_kw: float
    n_samples: int
    n_evaluations: int


def calibrate(
    p: ModelParams,
    drives: list[MeasuredDrive],
    params: tuple[CalibParam, ...] = DEFAULT_PARAMS,
) -> CalibrationResult:
    keys = [c.key for c in params]
    x0_abs = np.array([float(get_value(p, k)) for k in keys])
    scale = np.where(x0_abs != 0, np.abs(x0_abs), 1.0)
    lower = np.array([c.lower for c in params]) / scale
    upper = np.array([c.upper for c in params]) / scale
    x0 = np.clip(x0_abs / scale, lower, upper)

    def apply(x: np.ndarray) -> ModelParams:
        return with_changes(p, dict(zip(keys, x * scale, strict=True)))

    def residual(x: np.ndarray) -> np.ndarray:
        q = apply(x)
        parts = [sample_residual_w(simulate_drive(q, d), d) for d in drives]
        return np.concatenate(parts) / 1000.0  # kW keeps the cost well scaled

    r0 = residual(x0)
    sol = least_squares(residual, x0, bounds=(lower, upper), x_scale="jac", diff_step=1e-3)

    n, n_p = sol.fun.size, len(keys)
    s2 = float(np.sum(sol.fun**2) / max(n - n_p, 1))
    jtj = sol.jac.T @ sol.jac
    cov = s2 * np.linalg.pinv(jtj)
    std = np.sqrt(np.clip(np.diag(cov), 0, None)) * scale
    d = np.sqrt(np.clip(np.diag(cov), 1e-300, None))
    corr = cov / np.outer(d, d)

    est = sol.x * scale
    table = pd.DataFrame(
        {
            "parameter": keys,
            "initial": x0_abs,
            "estimate": est,
            "std": std,
            "ci95_low": est - 1.96 * std,
            "ci95_high": est + 1.96 * std,
            "at_bound": (np.isclose(sol.x, lower) | np.isclose(sol.x, upper)),
        }
    )
    short = [k.split(".")[1] for k in keys]
    return CalibrationResult(
        params=apply(sol.x),
        table=table,
        correlation=pd.DataFrame(corr, index=short, columns=short),
        rmse_before_kw=float(np.sqrt(np.mean(r0**2))),
        rmse_after_kw=float(np.sqrt(np.mean(sol.fun**2))),
        n_samples=n,
        n_evaluations=int(sol.nfev),
    )
