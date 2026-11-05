// Longitudinal road-load forces. Positive force opposes forward motion.
#pragma once

#include "evdynasim/params.hpp"

namespace evds {

double aero_drag_n(const ModelParams& p, double speed_mps);
double rolling_resistance_n(const ModelParams& p, double speed_mps, double grade_pct);
double grade_force_n(const ModelParams& p, double grade_pct);
double inertial_force_n(const ModelParams& p, double accel_mps2);

}  // namespace evds
