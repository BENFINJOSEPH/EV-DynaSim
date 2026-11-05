#include "evdynasim/powertrain.hpp"

#include <algorithm>
#include <cmath>

namespace evds {

double motor_loss_w(const ModelParams& p, double speed_rads, double torque_nm, bool motoring) {
    const Motor& m = p.motor;
    if (m.loss_map) return (*m.loss_map)(speed_rads * kRadsToRpm, std::abs(torque_nm));
    const double p_mech = std::abs(torque_nm) * speed_rads;
    return motoring ? p_mech * (1.0 / m.constant_efficiency - 1.0) : p_mech * (1.0 - m.constant_regen_efficiency);
}

PowertrainPoint compute_powertrain(const ModelParams& p, double f, double v) {
    const double r = p.body.wheel_radius_m;
    const double g = p.drivetrain.gear_ratio;
    const double eta_gb = p.drivetrain.gearbox_efficiency;

    PowertrainPoint out;
    const double w = v / r * g;
    const double p_wheel = f * v;
    const bool motoring = p_wheel > 0.0;
    const bool braking = p_wheel < 0.0;
    const double t_env = p.motor.max_torque_at(w);
    const double safe_w = w > 0.0 ? w : 1.0;
    out.motor_speed_rads = w;

    // motoring: wheel -> gearbox -> motor
    const double p_mech_drive = motoring ? p_wheel / eta_gb : 0.0;
    const double t_drive = motoring ? p_mech_drive / safe_w : 0.0;

    // braking: split between regen and friction brakes
    const double p_brake = braking ? -p_wheel : 0.0;
    double regen = std::min(p.brakes.regen_fraction * p_brake, p.brakes.max_regen_power_w);
    if (!(v >= p.brakes.min_regen_speed_mps)) regen = 0.0;
    regen = std::min(regen, t_env * w / eta_gb);
    double p_mech_regen = regen * eta_gb;
    double t_regen = braking ? p_mech_regen / safe_w : 0.0;

    const double loss_drive = motoring ? motor_loss_w(p, w, t_drive, true) : 0.0;
    double loss_regen = (braking && regen > 0.0) ? motor_loss_w(p, w, t_regen, false) : 0.0;
    if (braking && p_mech_regen <= loss_regen) {  // regen would cost energy: friction only
        regen = 0.0;
        p_mech_regen = 0.0;
        t_regen = 0.0;
        loss_regen = 0.0;
    }

    out.motor_torque_nm = t_drive - t_regen;
    out.motor_elec_w = (p_mech_drive + loss_drive) - (p_mech_regen - loss_regen);
    out.gearbox_loss_w = (p_mech_drive - (motoring ? p_wheel : 0.0)) + (regen - p_mech_regen);
    out.motor_loss_w = loss_drive + loss_regen;
    out.friction_brake_w = p_brake - regen;
    out.regen_wheel_w = regen;
    out.torque_limited = (motoring && t_drive > t_env * (1 + 1e-12)) || (w > p.motor.max_speed_rads);
    return out;
}

}  // namespace evds
