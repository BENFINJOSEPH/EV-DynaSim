// Model parameters, one struct per physical component (SI units).
// Mirrors src/evdynasim/params.py. Unit conversion from the configuration
// file (kW, km/h, rpm, mOhm) happens only in load_params().
#pragma once

#include <optional>
#include <string>

#include "evdynasim/interp.hpp"

namespace evds {

constexpr double kPi = 3.14159265358979323846;
constexpr double kKmhToMps = 1.0 / 3.6;
constexpr double kKwToW = 1000.0;
constexpr double kRpmToRads = 2.0 * kPi / 60.0;
constexpr double kRadsToRpm = 60.0 / (2.0 * kPi);
constexpr double kZeroCelsiusInK = 273.15;
constexpr double kJToKwh = 1.0 / 3.6e6;

struct Body {
    double mass_kg;
    double rotating_mass_factor;
    double drag_coefficient;
    double frontal_area_m2;
    double rolling_coefficient;
    double wheel_radius_m;
    double effective_mass_kg() const { return mass_kg * rotating_mass_factor; }
};

struct Environment {
    double air_density_kg_m3;
    double gravity_m_s2;
    double ambient_temp_c;
};

struct Drivetrain {
    double gear_ratio;
    double gearbox_efficiency;
};

struct Motor {
    double max_torque_nm;
    double max_power_w;
    double max_speed_rads;
    double constant_efficiency;
    double constant_regen_efficiency;
    std::optional<Table2D> loss_map;  // speed in rpm x |torque| in Nm -> loss in W

    double max_torque_at(double speed_rads) const;
};

struct Brakes {
    double regen_fraction;
    double max_regen_power_w;
    double min_regen_speed_mps;
};

struct Auxiliaries {
    double base_power_w;
    double hvac_w_per_k;
    double hvac_setpoint_c;
    double power_w(double ambient_temp_c) const;
};

struct Battery {
    int cells_series;
    int cells_parallel;
    double cell_capacity_ah;
    Table1D ocv_cell;  // SOC -> cell OCV in V
    double r0_cell_ohm;
    double r1_cell_ohm;
    double tau1_s;
    double arrhenius_b_k;
    double r_ref_temp_c;
    double max_charge_power_w;
    double max_discharge_power_w;
    double charge_derate_start_soc;
    double initial_soc;

    double capacity_ah() const { return cell_capacity_ah * cells_parallel; }
    double ocv_pack_v(double soc) const { return cells_series * ocv_cell(soc); }
    double resistance_factor(double temp_c) const;
    double r0_pack_ohm(double temp_c) const;
    double r1_pack_ohm(double temp_c) const;
    double charge_limit_w(double soc) const;
};

struct Thermal {
    double mass_kg;
    double heat_capacity_j_kgk;
    double h_a_w_k;
    double initial_temp_c;
};

struct ModelParams {
    std::string name;
    Body body;
    Environment env;
    Drivetrain drivetrain;
    Motor motor;
    Brakes brakes;
    Auxiliaries aux;
    Battery battery;
    Thermal thermal;

    // Throws std::invalid_argument with a clear message if a value is out of range.
    void validate() const;
};

ModelParams load_params(const std::string& config_path);

}  // namespace evds
