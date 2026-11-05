#include "evdynasim/params.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "evdynasim/config.hpp"

namespace evds {

double Motor::max_torque_at(double speed_rads) const {
    const double w = std::max(speed_rads, 1e-9);
    return std::min(max_torque_nm, max_power_w / w);
}

double Auxiliaries::power_w(double ambient_temp_c) const {
    return base_power_w + hvac_w_per_k * std::abs(ambient_temp_c - hvac_setpoint_c);
}

double Battery::resistance_factor(double temp_c) const {
    const double t = temp_c + kZeroCelsiusInK;
    const double t_ref = r_ref_temp_c + kZeroCelsiusInK;
    return std::exp(arrhenius_b_k * (1.0 / t - 1.0 / t_ref));
}

double Battery::r0_pack_ohm(double temp_c) const {
    return r0_cell_ohm * cells_series / cells_parallel * resistance_factor(temp_c);
}

double Battery::r1_pack_ohm(double temp_c) const {
    return r1_cell_ohm * cells_series / cells_parallel * resistance_factor(temp_c);
}

double Battery::charge_limit_w(double soc) const {
    const double frac = (1.0 - soc) / (1.0 - charge_derate_start_soc);
    return max_charge_power_w * std::min(std::max(frac, 0.0), 1.0);
}

namespace {

void require(bool ok, const std::string& what) {
    if (!ok) throw std::invalid_argument("invalid parameter: " + what);
}

}  // namespace

void ModelParams::validate() const {
    require(body.mass_kg > 0, "vehicle.mass_kg must be > 0");
    require(body.rotating_mass_factor >= 1.0, "vehicle.rotating_mass_factor must be >= 1");
    require(body.drag_coefficient >= 0, "vehicle.drag_coefficient must be >= 0");
    require(body.frontal_area_m2 > 0, "vehicle.frontal_area_m2 must be > 0");
    require(body.rolling_coefficient >= 0, "vehicle.rolling_coefficient must be >= 0");
    require(body.wheel_radius_m > 0, "vehicle.wheel_radius_m must be > 0");
    require(env.air_density_kg_m3 > 0 && env.gravity_m_s2 > 0, "environment values must be > 0");
    require(drivetrain.gear_ratio > 0, "drivetrain.gear_ratio must be > 0");
    require(drivetrain.gearbox_efficiency > 0 && drivetrain.gearbox_efficiency <= 1,
            "drivetrain.gearbox_efficiency must be in (0, 1]");
    require(motor.max_torque_nm > 0 && motor.max_power_w > 0 && motor.max_speed_rads > 0,
            "motor limits must be > 0");
    require(motor.constant_efficiency > 0 && motor.constant_efficiency <= 1,
            "motor.constant_efficiency must be in (0, 1]");
    require(motor.constant_regen_efficiency > 0 && motor.constant_regen_efficiency <= 1,
            "motor.constant_regen_efficiency must be in (0, 1]");
    require(brakes.regen_fraction >= 0 && brakes.regen_fraction <= 1, "brakes.regen_fraction must be in [0, 1]");
    require(brakes.max_regen_power_w >= 0 && brakes.min_regen_speed_mps >= 0, "brake limits must be >= 0");
    require(aux.base_power_w >= 0 && aux.hvac_w_per_k >= 0, "auxiliary power must be >= 0");
    require(battery.cells_series > 0 && battery.cells_parallel > 0, "battery cell counts must be > 0");
    require(battery.cell_capacity_ah > 0 && battery.tau1_s > 0, "battery capacity and tau1 must be > 0");
    require(battery.r0_cell_ohm >= 0 && battery.r1_cell_ohm >= 0, "battery resistances must be >= 0");
    require(battery.max_charge_power_w > 0 && battery.max_discharge_power_w > 0, "battery power limits must be > 0");
    require(battery.charge_derate_start_soc >= 0 && battery.charge_derate_start_soc < 1,
            "battery.charge_derate_start_soc must be in [0, 1)");
    require(battery.initial_soc >= 0 && battery.initial_soc <= 1, "battery.initial_soc must be in [0, 1]");
    require(thermal.mass_kg > 0 && thermal.heat_capacity_j_kgk > 0 && thermal.h_a_w_k >= 0,
            "thermal values must be > 0");
}

ModelParams load_params(const std::string& config_path) {
    const Config c = Config::load(config_path);
    ModelParams p;
    p.name = c.text("name", "unnamed vehicle");

    p.body = {c.number("vehicle.mass_kg"),          c.number("vehicle.rotating_mass_factor"),
              c.number("vehicle.drag_coefficient"), c.number("vehicle.frontal_area_m2"),
              c.number("vehicle.rolling_coefficient"), c.number("vehicle.wheel_radius_m")};
    p.env = {c.number("environment.air_density_kg_m3"), c.number("environment.gravity_m_s2"),
             c.number("environment.ambient_temp_c")};
    p.drivetrain = {c.number("drivetrain.gear_ratio"), c.number("drivetrain.gearbox_efficiency")};

    p.motor.max_torque_nm = c.number("motor.max_torque_nm");
    p.motor.max_power_w = c.number("motor.max_power_kw") * kKwToW;
    p.motor.max_speed_rads = c.number("motor.max_speed_rpm") * kRpmToRads;
    p.motor.constant_efficiency = c.number("motor.constant_efficiency");
    p.motor.constant_regen_efficiency = c.number("motor.constant_regen_efficiency");
    const std::string map_file = c.text("motor.loss_map_csv");
    if (!map_file.empty()) p.motor.loss_map = Table2D::from_csv(c.base_dir() + "/" + map_file);

    p.brakes = {c.number("brakes.regen_fraction"), c.number("brakes.max_regen_power_kw") * kKwToW,
                c.number("brakes.min_regen_speed_kmh") * kKmhToMps};
    p.aux = {c.number("auxiliaries.base_power_kw") * kKwToW, c.number("auxiliaries.hvac_kw_per_k") * kKwToW,
             c.number("auxiliaries.hvac_setpoint_c")};

    Battery& b = p.battery;
    b.cells_series = static_cast<int>(c.number("battery.cells_series"));
    b.cells_parallel = static_cast<int>(c.number("battery.cells_parallel"));
    b.cell_capacity_ah = c.number("battery.cell_capacity_ah");
    b.ocv_cell = Table1D(c.list("battery.ocv_soc"), c.list("battery.ocv_cell_v"));
    b.r0_cell_ohm = c.number("battery.r0_cell_mohm") / 1000.0;
    b.r1_cell_ohm = c.number("battery.r1_cell_mohm") / 1000.0;
    b.tau1_s = c.number("battery.tau1_s");
    b.arrhenius_b_k = c.number("battery.arrhenius_b_k");
    b.r_ref_temp_c = c.number("battery.r_ref_temp_c");
    b.max_charge_power_w = c.number("battery.max_charge_power_kw") * kKwToW;
    b.max_discharge_power_w = c.number("battery.max_discharge_power_kw") * kKwToW;
    b.charge_derate_start_soc = c.number("battery.charge_derate_start_soc");
    b.initial_soc = c.number("battery.initial_soc");

    p.thermal = {c.number("thermal.mass_kg"), c.number("thermal.heat_capacity_j_kgk"),
                 c.number("thermal.h_a_w_k"), c.number("thermal.initial_temp_c")};
    p.validate();
    return p;
}

}  // namespace evds
