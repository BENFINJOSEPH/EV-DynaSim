// Backward-facing simulation over a drive cycle. Same algorithm as
// src/evdynasim/simulate.py: vehicle + powertrain per interval, then the
// battery with charge-limit curtailment of regen.
#pragma once

#include <vector>

#include "evdynasim/drive_cycle.hpp"
#include "evdynasim/params.hpp"

namespace evds {

struct Summary {
    double duration_s = 0, distance_km = 0, mean_speed_kmh = 0, max_speed_kmh = 0;
    double battery_energy_kwh = 0, consumption_kwh_per_100km = 0, chemical_energy_kwh = 0;
    double regen_energy_kwh = 0, regen_curtailed_kwh = 0;
    double aero_kwh = 0, rolling_kwh = 0, grade_kwh = 0, kinetic_change_kwh = 0;
    double gearbox_loss_kwh = 0, motor_loss_kwh = 0, friction_brake_kwh = 0, aux_kwh = 0, battery_loss_kwh = 0;
    double peak_battery_power_kw = 0, peak_regen_power_kw = 0, max_current_a = 0, min_terminal_voltage_v = 0;
    double initial_soc = 0, final_soc = 0, max_temp_c = 0, final_temp_c = 0;
    int torque_limited_steps = 0, battery_limited_steps = 0;

    double sink_total_kwh() const {
        return aero_kwh + rolling_kwh + grade_kwh + kinetic_change_kwh + gearbox_loss_kwh + motor_loss_kwh +
               friction_brake_kwh + aux_kwh + battery_loss_kwh;
    }
};

// One row per interval (N-1 rows); soc and temperature per sample (N values).
struct SimulationResult {
    std::vector<double> t_start_s, dt_s, speed_mps, accel_mps2, wheel_power_w;
    std::vector<double> motor_speed_rads, motor_torque_nm, battery_power_w, current_a, terminal_v;
    std::vector<double> soc, temp_c;
    Summary summary;
};

SimulationResult simulate(const ModelParams& p, const DriveCycle& cycle);

}  // namespace evds
