#include "evdynasim/physics.hpp"

#include <cmath>
#include <stdexcept>

namespace evds {
namespace {
double grade_angle_rad(double grade_pct) { return std::atan(grade_pct / 100.0); }
}  // namespace

double aero_drag_n(const ModelParams& p, double v) {
    if (v < 0.0) throw std::invalid_argument("speed must not be negative");
    return 0.5 * p.env.air_density_kg_m3 * p.body.drag_coefficient * p.body.frontal_area_m2 * (v * v);
}

double rolling_resistance_n(const ModelParams& p, double v, double grade_pct) {
    if (v < 0.0) throw std::invalid_argument("speed must not be negative");
    if (!(v > 0.0)) return 0.0;  // no rolling loss at standstill
    return p.body.mass_kg * p.env.gravity_m_s2 * p.body.rolling_coefficient * std::cos(grade_angle_rad(grade_pct));
}

double grade_force_n(const ModelParams& p, double grade_pct) {
    return p.body.mass_kg * p.env.gravity_m_s2 * std::sin(grade_angle_rad(grade_pct));
}

double inertial_force_n(const ModelParams& p, double accel_mps2) { return p.body.effective_mass_kg() * accel_mps2; }

}  // namespace evds
