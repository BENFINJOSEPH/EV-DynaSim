#include "evdynasim/battery.hpp"

#include <cmath>

namespace evds {

BatteryState initial_battery_state(const ModelParams& p) {
    return {p.battery.initial_soc, 0.0, p.thermal.initial_temp_c};
}

CurrentSolution current_for_power(double e, double r0, double power) {
    const double disc = e * e - 4.0 * r0 * power;
    if (disc < 0.0) return {e / (2.0 * r0), true};
    return {2.0 * power / (e + std::sqrt(disc)), false};
}

BatteryStepInfo battery_step(const ModelParams& p, BatteryState& s, double terminal_power_w, double dt) {
    const Battery& bat = p.battery;
    const Thermal& th = p.thermal;
    const double ocv = bat.ocv_pack_v(s.soc);
    const double r0 = bat.r0_pack_ohm(s.temp_c);
    const double r1 = bat.r1_pack_ohm(s.temp_c);

    bool limited = terminal_power_w > bat.max_discharge_power_w;
    const CurrentSolution sol = current_for_power(ocv - s.u1_v, r0, terminal_power_w);
    limited = limited || sol.beyond_max_power;
    const double i = sol.current_a;

    BatteryStepInfo info;
    info.current_a = i;
    info.terminal_v = ocv - s.u1_v - r0 * i;
    info.ocv_v = ocv;
    info.loss_w = i * i * r0 + s.u1_v * i;
    info.heat_w = i * i * r0 + (r1 > 0.0 ? s.u1_v * s.u1_v / r1 : 0.0);
    info.power_limited = limited;

    const double decay = std::exp(-dt / bat.tau1_s);
    const double u1_next = s.u1_v * decay + r1 * (1.0 - decay) * i;
    const double soc_next = s.soc - i * dt / (3600.0 * bat.capacity_ah());
    const double q_out = th.h_a_w_k * (s.temp_c - p.env.ambient_temp_c);
    const double temp_next = s.temp_c + dt * (info.heat_w - q_out) / (th.mass_kg * th.heat_capacity_j_kgk);

    s = {soc_next, u1_next, temp_next};
    return info;
}

}  // namespace evds
