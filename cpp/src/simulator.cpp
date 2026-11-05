#include "evdynasim/simulator.hpp"

#include <algorithm>
#include <cmath>

#include "evdynasim/battery.hpp"
#include "evdynasim/physics.hpp"
#include "evdynasim/powertrain.hpp"

namespace evds {

SimulationResult simulate(const ModelParams& p, const DriveCycle& cycle) {
    cycle.validate();
    const std::size_t n = cycle.time_s.size() - 1;
    const double aux_w = p.aux.power_w(p.env.ambient_temp_c);

    SimulationResult r;
    r.soc.reserve(n + 1);
    r.temp_c.reserve(n + 1);
    Summary& s = r.summary;

    // energies accumulated in joules
    double e_batt = 0, e_chem = 0, e_regen = 0, e_curtailed = 0, e_aero = 0, e_roll = 0, e_grade = 0;
    double e_kin = 0, e_gb = 0, e_motor = 0, e_friction = 0, e_aux = 0, e_bloss = 0, dist_m = 0;
    double peak_batt = -1e300, min_batt = 1e300, max_abs_i = 0, min_u = 1e300;

    BatteryState state = initial_battery_state(p);
    r.soc.push_back(state.soc);
    r.temp_c.push_back(state.temp_c);

    for (std::size_t k = 0; k < n; ++k) {
        const double dt = cycle.time_s[k + 1] - cycle.time_s[k];
        const double vbar = 0.5 * (cycle.speed_mps[k] + cycle.speed_mps[k + 1]);
        const double accel = (cycle.speed_mps[k + 1] - cycle.speed_mps[k]) / dt;
        const double grade = 0.5 * (cycle.grade_pct[k] + cycle.grade_pct[k + 1]);

        // 1. vehicle and powertrain
        const double f_aero = aero_drag_n(p, vbar);
        const double f_roll = rolling_resistance_n(p, vbar, grade);
        const double f_grade = vbar > 0.0 ? grade_force_n(p, grade) : 0.0;
        const double f_inertia = inertial_force_n(p, accel);
        const double f_wheel = f_aero + f_roll + f_grade + f_inertia;
        PowertrainPoint pt = compute_powertrain(p, f_wheel, vbar);

        // 2. battery, with charge-limit curtailment of regen
        double p_req = pt.motor_elec_w + aux_w;
        const double limit = p.battery.charge_limit_w(state.soc);
        if (p_req < -limit && pt.motor_elec_w < 0.0) {
            const double scale = std::max(0.0, 1.0 - (-limit - p_req) / (-pt.motor_elec_w));
            e_curtailed += (1.0 - scale) * pt.regen_wheel_w * dt;
            pt.friction_brake_w += (1.0 - scale) * pt.regen_wheel_w;
            pt.regen_wheel_w *= scale;
            pt.motor_elec_w *= scale;
            pt.motor_loss_w *= scale;
            pt.gearbox_loss_w *= scale;
            p_req = pt.motor_elec_w + aux_w;
        }
        const BatteryStepInfo b = battery_step(p, state, p_req, dt);
        const double p_batt = b.terminal_v * b.current_a;

        // 3. record and accumulate
        r.t_start_s.push_back(cycle.time_s[k]);
        r.dt_s.push_back(dt);
        r.speed_mps.push_back(vbar);
        r.accel_mps2.push_back(accel);
        r.wheel_power_w.push_back(f_wheel * vbar);
        r.motor_speed_rads.push_back(pt.motor_speed_rads);
        r.motor_torque_nm.push_back(pt.motor_torque_nm);
        r.battery_power_w.push_back(p_batt);
        r.current_a.push_back(b.current_a);
        r.terminal_v.push_back(b.terminal_v);
        r.soc.push_back(state.soc);
        r.temp_c.push_back(state.temp_c);

        dist_m += vbar * dt;
        e_batt += p_batt * dt;
        e_chem += b.ocv_v * b.current_a * dt;
        e_regen += std::max(-pt.motor_elec_w, 0.0) * dt;
        e_aero += f_aero * vbar * dt;
        e_roll += f_roll * vbar * dt;
        e_grade += f_grade * vbar * dt;
        e_kin += f_inertia * vbar * dt;
        e_gb += pt.gearbox_loss_w * dt;
        e_motor += pt.motor_loss_w * dt;
        e_friction += pt.friction_brake_w * dt;
        e_aux += aux_w * dt;
        e_bloss += b.loss_w * dt;
        peak_batt = std::max(peak_batt, p_batt);
        min_batt = std::min(min_batt, p_batt);
        max_abs_i = std::max(max_abs_i, std::abs(b.current_a));
        min_u = std::min(min_u, b.terminal_v);
        s.torque_limited_steps += pt.torque_limited ? 1 : 0;
        s.battery_limited_steps += b.power_limited ? 1 : 0;
    }

    s.duration_s = cycle.duration_s();
    s.distance_km = dist_m / 1000.0;
    s.mean_speed_kmh = s.distance_km / (s.duration_s / 3600.0);
    s.max_speed_kmh = *std::max_element(cycle.speed_mps.begin(), cycle.speed_mps.end()) * 3.6;
    s.battery_energy_kwh = e_batt * kJToKwh;
    s.consumption_kwh_per_100km = s.distance_km > 0 ? 100.0 * s.battery_energy_kwh / s.distance_km : NAN;
    s.chemical_energy_kwh = e_chem * kJToKwh;
    s.regen_energy_kwh = e_regen * kJToKwh;
    s.regen_curtailed_kwh = e_curtailed * kJToKwh;
    s.aero_kwh = e_aero * kJToKwh;
    s.rolling_kwh = e_roll * kJToKwh;
    s.grade_kwh = e_grade * kJToKwh;
    s.kinetic_change_kwh = e_kin * kJToKwh;
    s.gearbox_loss_kwh = e_gb * kJToKwh;
    s.motor_loss_kwh = e_motor * kJToKwh;
    s.friction_brake_kwh = e_friction * kJToKwh;
    s.aux_kwh = e_aux * kJToKwh;
    s.battery_loss_kwh = e_bloss * kJToKwh;
    s.peak_battery_power_kw = peak_batt / 1e3;
    s.peak_regen_power_kw = std::max(0.0, -min_batt) / 1e3;
    s.max_current_a = max_abs_i;
    s.min_terminal_voltage_v = min_u;
    s.initial_soc = r.soc.front();
    s.final_soc = r.soc.back();
    s.max_temp_c = *std::max_element(r.temp_c.begin(), r.temp_c.end());
    s.final_temp_c = r.temp_c.back();
    return r;
}

}  // namespace evds
