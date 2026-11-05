// Wheel force/speed -> motor operating point -> electrical power.
// Same equations as src/evdynasim/powertrain.py (loss-map formulation).
#pragma once

#include "evdynasim/params.hpp"

namespace evds {

struct PowertrainPoint {
    double motor_speed_rads = 0.0;
    double motor_torque_nm = 0.0;   // + motoring, - regen
    double motor_elec_w = 0.0;      // + from battery, - to battery
    double gearbox_loss_w = 0.0;
    double motor_loss_w = 0.0;
    double friction_brake_w = 0.0;
    double regen_wheel_w = 0.0;     // braking power taken by the motor at the wheel
    bool torque_limited = false;
};

double motor_loss_w(const ModelParams& p, double speed_rads, double torque_nm, bool motoring);
PowertrainPoint compute_powertrain(const ModelParams& p, double wheel_force_n, double speed_mps);

}  // namespace evds
