// 1-RC equivalent circuit battery with lumped thermal model.
// Same discretisation as src/evdynasim/battery.py.
#pragma once

#include "evdynasim/params.hpp"

namespace evds {

struct BatteryState {
    double soc;
    double u1_v;
    double temp_c;
};

struct BatteryStepInfo {
    double current_a;
    double terminal_v;
    double ocv_v;
    double loss_w;   // I^2 R0 + U1 I
    double heat_w;   // I^2 R0 + U1^2 / R1
    bool power_limited;
};

struct CurrentSolution {
    double current_a;
    bool beyond_max_power;
};

BatteryState initial_battery_state(const ModelParams& p);

// Solve P = (E - R0 I) I for the smaller root (stable form, valid for R0 = 0).
CurrentSolution current_for_power(double e_v, double r0_ohm, double power_w);

// Advance one step with constant terminal power. Returns info, updates state.
BatteryStepInfo battery_step(const ModelParams& p, BatteryState& state, double terminal_power_w, double dt_s);

}  // namespace evds
