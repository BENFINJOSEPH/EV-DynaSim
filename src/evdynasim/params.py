"""Model parameters: immutable, validated, SI units inside.

Design
------
* One frozen dataclass per physical component (body, motor, battery, ...)
  and one ``ModelParams`` that groups them. The C++ engine uses the same
  structure (one ``struct`` per component).
* The YAML file uses engineering units (kW, kWh, km/h, rpm, degC, mOhm).
  Conversion to SI happens exactly once, in ``ModelParams.from_dict``.
  Temperatures stay in degC in the parameter objects and are converted to
  Kelvin only where physics needs it (Arrhenius term).
* ``with_changes(p, {"body.mass_kg": 2000})`` creates a modified copy. It is
  used by the sensitivity study and by the calibration.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from evdynasim.interp import Table1D, Table2D

KMH_TO_MPS = 1.0 / 3.6
KW_TO_W = 1000.0
KWH_TO_J = 3.6e6
RPM_TO_RADS = 2.0 * np.pi / 60.0
ZERO_C_IN_K = 273.15


def _check(obj: Any, positive=(), non_negative=(), fraction=(), efficiency=()) -> None:
    """Raise ValueError with a clear message for out-of-range parameters."""
    cls = type(obj).__name__
    for name in positive:
        if not getattr(obj, name) > 0:
            raise ValueError(f"{cls}.{name} must be > 0, got {getattr(obj, name)}")
    for name in non_negative:
        if not getattr(obj, name) >= 0:
            raise ValueError(f"{cls}.{name} must be >= 0, got {getattr(obj, name)}")
    for name in fraction:
        if not 0.0 <= getattr(obj, name) <= 1.0:
            raise ValueError(f"{cls}.{name} must be in [0, 1], got {getattr(obj, name)}")
    for name in efficiency:
        if not 0.0 < getattr(obj, name) <= 1.0:
            raise ValueError(f"{cls}.{name} must be in (0, 1], got {getattr(obj, name)}")


@dataclass(frozen=True)
class Body:
    mass_kg: float
    rotating_mass_factor: float
    drag_coefficient: float
    frontal_area_m2: float
    rolling_coefficient: float
    wheel_radius_m: float

    def __post_init__(self) -> None:
        _check(self, positive=("mass_kg", "frontal_area_m2", "wheel_radius_m"),
               non_negative=("drag_coefficient", "rolling_coefficient"))  # fmt: skip
        if self.rotating_mass_factor < 1.0:
            raise ValueError("Body.rotating_mass_factor must be >= 1.0")

    @property
    def effective_mass_kg(self) -> float:
        return self.mass_kg * self.rotating_mass_factor


@dataclass(frozen=True)
class Environment:
    air_density_kg_m3: float
    gravity_m_s2: float
    ambient_temp_c: float

    def __post_init__(self) -> None:
        _check(self, positive=("air_density_kg_m3", "gravity_m_s2"))


@dataclass(frozen=True)
class Drivetrain:
    gear_ratio: float
    gearbox_efficiency: float

    def __post_init__(self) -> None:
        _check(self, positive=("gear_ratio",), efficiency=("gearbox_efficiency",))


@dataclass(frozen=True)
class Motor:
    """Traction motor + inverter.

    Losses come from a loss map P_loss(speed_rpm, |torque|) if one is given,
    otherwise from constant efficiencies (simple model, used for hand checks).
    """

    max_torque_nm: float
    max_power_w: float
    max_speed_rads: float
    constant_efficiency: float
    constant_regen_efficiency: float
    loss_map: Table2D | None = None  # x = speed in rpm, y = |torque| in Nm, z = loss in W

    def __post_init__(self) -> None:
        _check(self, positive=("max_torque_nm", "max_power_w", "max_speed_rads"),
               efficiency=("constant_efficiency", "constant_regen_efficiency"))  # fmt: skip

    def max_torque_at(self, speed_rads):
        """Torque envelope: constant torque below base speed, constant power above."""
        w = np.maximum(np.asarray(speed_rads, dtype=float), 1e-9)
        return np.minimum(self.max_torque_nm, self.max_power_w / w)


@dataclass(frozen=True)
class Brakes:
    regen_fraction: float
    max_regen_power_w: float
    min_regen_speed_mps: float

    def __post_init__(self) -> None:
        _check(self, fraction=("regen_fraction",),
               non_negative=("max_regen_power_w", "min_regen_speed_mps"))  # fmt: skip


@dataclass(frozen=True)
class Auxiliaries:
    """P_aux = base + hvac_w_per_k * |T_ambient - T_setpoint|."""

    base_power_w: float
    hvac_w_per_k: float
    hvac_setpoint_c: float

    def __post_init__(self) -> None:
        _check(self, non_negative=("base_power_w", "hvac_w_per_k"))

    def power_w(self, ambient_temp_c: float) -> float:
        return self.base_power_w + self.hvac_w_per_k * abs(ambient_temp_c - self.hvac_setpoint_c)


@dataclass(frozen=True)
class Battery:
    """Pack built from identical cells, 1-RC equivalent circuit model."""

    cells_series: int
    cells_parallel: int
    cell_capacity_ah: float
    ocv_cell: Table1D  # x = SOC (0..1), y = cell open-circuit voltage in V
    r0_cell_ohm: float
    r1_cell_ohm: float
    tau1_s: float
    arrhenius_b_k: float  # R(T) = R_ref * exp(B * (1/T - 1/T_ref))
    r_ref_temp_c: float
    max_charge_power_w: float
    max_discharge_power_w: float
    charge_derate_start_soc: float  # charge limit falls linearly to 0 between this SOC and 1
    initial_soc: float

    def __post_init__(self) -> None:
        _check(self, positive=("cells_series", "cells_parallel", "cell_capacity_ah", "tau1_s",
                               "max_charge_power_w", "max_discharge_power_w"),
               non_negative=("r0_cell_ohm", "r1_cell_ohm", "arrhenius_b_k"),
               fraction=("charge_derate_start_soc", "initial_soc"))  # fmt: skip
        if self.charge_derate_start_soc >= 1.0:
            raise ValueError("Battery.charge_derate_start_soc must be < 1")

    @property
    def capacity_ah(self) -> float:
        return self.cell_capacity_ah * self.cells_parallel

    @property
    def nominal_energy_kwh(self) -> float:
        """Capacity times OCV averaged over SOC 0..1 (trapezoid on the table)."""
        mean_ocv = np.trapezoid(self.ocv_cell.y, self.ocv_cell.x) / (
            self.ocv_cell.x[-1] - self.ocv_cell.x[0]
        )
        return self.capacity_ah * mean_ocv * self.cells_series / 1000.0

    def ocv_pack_v(self, soc):
        return self.cells_series * self.ocv_cell(soc)

    def resistance_factor(self, temp_c):
        t = np.asarray(temp_c, dtype=float) + ZERO_C_IN_K
        t_ref = self.r_ref_temp_c + ZERO_C_IN_K
        return np.exp(self.arrhenius_b_k * (1.0 / t - 1.0 / t_ref))

    def r0_pack_ohm(self, temp_c):
        return (
            self.r0_cell_ohm
            * self.cells_series
            / self.cells_parallel
            * self.resistance_factor(temp_c)
        )

    def r1_pack_ohm(self, temp_c):
        return (
            self.r1_cell_ohm
            * self.cells_series
            / self.cells_parallel
            * self.resistance_factor(temp_c)
        )

    def charge_limit_w(self, soc: float) -> float:
        frac = (1.0 - soc) / (1.0 - self.charge_derate_start_soc)
        return self.max_charge_power_w * min(max(frac, 0.0), 1.0)


@dataclass(frozen=True)
class Thermal:
    """Lumped battery thermal model: m c dT/dt = Q_gen - hA (T - T_amb)."""

    mass_kg: float
    heat_capacity_j_kgk: float
    h_a_w_k: float
    initial_temp_c: float

    def __post_init__(self) -> None:
        _check(self, positive=("mass_kg", "heat_capacity_j_kgk"), non_negative=("h_a_w_k",))


@dataclass(frozen=True)
class Charging:
    """DC fast charging: charger, battery management (BMS) limits and active cooling.

    The BMS allows the smallest of three currents (see charging.py):
    the C-rate limit (reduced by temperature), the charger power limit and
    the current that keeps the terminal voltage at the CV limit.
    """

    charger_max_power_w: float
    charger_efficiency: float  # grid -> battery terminals
    max_c_rate: float  # 1/h, at full temperature permission
    cv_cell_voltage_v: float  # constant-voltage limit per cell
    derate_temp_c: Table1D  # x = battery temperature, y = allowed fraction of max C-rate
    derate_soc: Table1D  # x = SOC, y = allowed fraction of max C-rate (taper at high SOC)
    max_cell_temp_c: float  # charging stops above this temperature
    cooling_w_k: float  # active cooling during charging
    coolant_temp_c: float
    end_current_c_rate: float  # CV phase ends when the current falls below this

    def __post_init__(self) -> None:
        _check(self, positive=("charger_max_power_w", "max_c_rate", "cv_cell_voltage_v",
                               "max_cell_temp_c"),
               efficiency=("charger_efficiency",),
               non_negative=("cooling_w_k", "end_current_c_rate"))  # fmt: skip
        for table in (self.derate_temp_c, self.derate_soc):
            if table.y.min() < 0.0 or table.y.max() > 1.0:
                raise ValueError("Charging derating factors must be in [0, 1]")

    def temperature_factor(self, temp_c: float) -> float:
        """Allowed fraction of the maximum C-rate at this battery temperature."""
        if temp_c >= self.max_cell_temp_c:
            return 0.0
        return float(self.derate_temp_c(temp_c))

    def soc_factor(self, soc: float) -> float:
        """Allowed fraction of the maximum C-rate at this SOC."""
        return float(self.derate_soc(soc))


@dataclass(frozen=True)
class ModelParams:
    name: str
    body: Body
    env: Environment
    drivetrain: Drivetrain
    motor: Motor
    brakes: Brakes
    aux: Auxiliaries
    battery: Battery
    thermal: Thermal
    charging: Charging | None = None  # optional section, only needed for charging studies

    @classmethod
    def from_dict(cls, cfg: dict[str, Any], base_dir: Path | None = None) -> ModelParams:
        base_dir = base_dir or Path.cwd()
        try:
            v, e, d, m = cfg["vehicle"], cfg["environment"], cfg["drivetrain"], cfg["motor"]
            b, a, bat, th = cfg["brakes"], cfg["auxiliaries"], cfg["battery"], cfg["thermal"]
            map_file = m.get("loss_map_csv") or ""
            loss_map = Table2D.from_csv(base_dir / map_file) if map_file else None
            return cls(
                name=str(cfg.get("name", "unnamed vehicle")),
                body=Body(
                    mass_kg=float(v["mass_kg"]),
                    rotating_mass_factor=float(v["rotating_mass_factor"]),
                    drag_coefficient=float(v["drag_coefficient"]),
                    frontal_area_m2=float(v["frontal_area_m2"]),
                    rolling_coefficient=float(v["rolling_coefficient"]),
                    wheel_radius_m=float(v["wheel_radius_m"]),
                ),
                env=Environment(
                    air_density_kg_m3=float(e["air_density_kg_m3"]),
                    gravity_m_s2=float(e["gravity_m_s2"]),
                    ambient_temp_c=float(e["ambient_temp_c"]),
                ),
                drivetrain=Drivetrain(
                    gear_ratio=float(d["gear_ratio"]),
                    gearbox_efficiency=float(d["gearbox_efficiency"]),
                ),
                motor=Motor(
                    max_torque_nm=float(m["max_torque_nm"]),
                    max_power_w=float(m["max_power_kw"]) * KW_TO_W,
                    max_speed_rads=float(m["max_speed_rpm"]) * RPM_TO_RADS,
                    constant_efficiency=float(m["constant_efficiency"]),
                    constant_regen_efficiency=float(m["constant_regen_efficiency"]),
                    loss_map=loss_map,
                ),
                brakes=Brakes(
                    regen_fraction=float(b["regen_fraction"]),
                    max_regen_power_w=float(b["max_regen_power_kw"]) * KW_TO_W,
                    min_regen_speed_mps=float(b["min_regen_speed_kmh"]) * KMH_TO_MPS,
                ),
                aux=Auxiliaries(
                    base_power_w=float(a["base_power_kw"]) * KW_TO_W,
                    hvac_w_per_k=float(a["hvac_kw_per_k"]) * KW_TO_W,
                    hvac_setpoint_c=float(a["hvac_setpoint_c"]),
                ),
                battery=Battery(
                    cells_series=int(bat["cells_series"]),
                    cells_parallel=int(bat["cells_parallel"]),
                    cell_capacity_ah=float(bat["cell_capacity_ah"]),
                    ocv_cell=Table1D(bat["ocv_soc"], bat["ocv_cell_v"]),
                    r0_cell_ohm=float(bat["r0_cell_mohm"]) / 1000.0,
                    r1_cell_ohm=float(bat["r1_cell_mohm"]) / 1000.0,
                    tau1_s=float(bat["tau1_s"]),
                    arrhenius_b_k=float(bat["arrhenius_b_k"]),
                    r_ref_temp_c=float(bat["r_ref_temp_c"]),
                    max_charge_power_w=float(bat["max_charge_power_kw"]) * KW_TO_W,
                    max_discharge_power_w=float(bat["max_discharge_power_kw"]) * KW_TO_W,
                    charge_derate_start_soc=float(bat["charge_derate_start_soc"]),
                    initial_soc=float(bat["initial_soc"]),
                ),
                thermal=Thermal(
                    mass_kg=float(th["mass_kg"]),
                    heat_capacity_j_kgk=float(th["heat_capacity_j_kgk"]),
                    h_a_w_k=float(th["h_a_w_k"]),
                    initial_temp_c=float(th["initial_temp_c"]),
                ),
                charging=_charging_from_dict(cfg.get("charging")),
            )
        except KeyError as exc:
            raise KeyError(f"Missing configuration key: {exc}") from exc


def _charging_from_dict(c: dict[str, Any] | None) -> Charging | None:
    if not c:
        return None
    return Charging(
        charger_max_power_w=float(c["charger_max_power_kw"]) * KW_TO_W,
        charger_efficiency=float(c["charger_efficiency"]),
        max_c_rate=float(c["max_c_rate"]),
        cv_cell_voltage_v=float(c["cv_cell_voltage_v"]),
        derate_temp_c=Table1D(c["derate_temp_c"], c["derate_temp_factor"]),
        derate_soc=Table1D(c["derate_soc"], c["derate_soc_factor"]),
        max_cell_temp_c=float(c["max_cell_temp_c"]),
        cooling_w_k=float(c["cooling_w_k"]),
        coolant_temp_c=float(c["coolant_temp_c"]),
        end_current_c_rate=float(c["end_current_c_rate"]),
    )


def load_params(path: str | Path) -> ModelParams:
    """Load a vehicle configuration. Relative file paths inside it (loss map)
    are resolved relative to the YAML file's folder."""
    path = Path(path)
    with path.open(encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    if not isinstance(cfg, dict):
        raise ValueError(f"{path} does not contain a YAML mapping")
    return ModelParams.from_dict(cfg, base_dir=path.resolve().parent)


def with_changes(p: ModelParams, changes: dict[str, Any]) -> ModelParams:
    """Return a copy with dotted-path changes, e.g. {"body.mass_kg": 2000}."""
    for key, value in changes.items():
        parts = key.split(".")
        if len(parts) == 1:
            p = dataclasses.replace(p, **{parts[0]: value})
        elif len(parts) == 2:
            comp = getattr(p, parts[0])
            p = dataclasses.replace(p, **{parts[0]: dataclasses.replace(comp, **{parts[1]: value})})
        else:
            raise ValueError(f"Unsupported parameter path: {key}")
    return p


def get_value(p: ModelParams, key: str) -> Any:
    obj: Any = p
    for part in key.split("."):
        obj = getattr(obj, part)
    return obj
