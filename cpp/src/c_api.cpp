#include "evdynasim/evdynasim_c.h"

#include <algorithm>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <vector>
#include <string>

#include "evdynasim/simulator.hpp"

namespace {

void write_error(char* buffer, size_t len, const std::string& msg) {
    if (buffer == nullptr || len == 0) return;
    const size_t count = std::min(len - 1, msg.size());
    std::memcpy(buffer, msg.data(), count);
    buffer[count] = '\0';
}

evds_summary to_c(const evds::Summary& s) {
    return {s.duration_s, s.distance_km, s.mean_speed_kmh, s.max_speed_kmh,
            s.battery_energy_kwh, s.consumption_kwh_per_100km, s.chemical_energy_kwh,
            s.regen_energy_kwh, s.regen_curtailed_kwh,
            s.aero_kwh, s.rolling_kwh, s.grade_kwh, s.kinetic_change_kwh,
            s.gearbox_loss_kwh, s.motor_loss_kwh, s.friction_brake_kwh, s.aux_kwh, s.battery_loss_kwh,
            s.peak_battery_power_kw, s.peak_regen_power_kw, s.max_current_a, s.min_terminal_voltage_v,
            s.initial_soc, s.final_soc, s.max_temp_c, s.final_temp_c,
            s.torque_limited_steps, s.battery_limited_steps};
}

}  // namespace

extern "C" {

const char* evds_version(void) { return "1.0.0"; }

int evds_simulate(const char* config_path, const double* time_s, const double* speed_mps, const double* grade_pct,
                  size_t n, double* battery_power_w, double* soc, double* temp_c, evds_summary* summary,
                  char* error_msg, size_t error_len) {
    try {
        if (config_path == nullptr || time_s == nullptr || speed_mps == nullptr || grade_pct == nullptr) {
            throw std::invalid_argument("null input pointer");
        }
        const evds::ModelParams params = evds::load_params(config_path);
        evds::DriveCycle cycle{"c_api", std::vector<double>(time_s, time_s + n),
                               std::vector<double>(speed_mps, speed_mps + n),
                               std::vector<double>(grade_pct, grade_pct + n)};
        const evds::SimulationResult r = evds::simulate(params, cycle);
        if (battery_power_w) std::copy(r.battery_power_w.begin(), r.battery_power_w.end(), battery_power_w);
        if (soc) std::copy(r.soc.begin(), r.soc.end(), soc);
        if (temp_c) std::copy(r.temp_c.begin(), r.temp_c.end(), temp_c);
        if (summary) *summary = to_c(r.summary);
        write_error(error_msg, error_len, "");
        return EVDS_OK;
    } catch (const std::exception& e) {
        write_error(error_msg, error_len, e.what());
        return EVDS_ERROR;
    } catch (...) {
        write_error(error_msg, error_len, "unknown error");
        return EVDS_ERROR;
    }
}

}  // extern "C"
