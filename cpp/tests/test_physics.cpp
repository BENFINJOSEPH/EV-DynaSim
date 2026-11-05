#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "evdynasim/params.hpp"
#include "evdynasim/physics.hpp"
#include "evdynasim/powertrain.hpp"

using namespace evds;

namespace {
ModelParams simple() { return load_params(std::string(EVDS_REPO_DIR) + "/configs/simple_ev.yaml"); }
ModelParams generic() { return load_params(std::string(EVDS_REPO_DIR) + "/configs/generic_ev.yaml"); }
const double kV100 = 100.0 / 3.6;
}  // namespace

// Same hand-calculated values as tests/test_physics.py and docs/03_physics_model.md
TEST(Physics, HandCalculationAt100kmh) {
    const ModelParams p = simple();
    EXPECT_NEAR(aero_drag_n(p, kV100), 304.36, 0.01);
    EXPECT_NEAR(rolling_resistance_n(p, kV100, 0.0), 176.58, 1e-9);
    const double f = aero_drag_n(p, kV100) + rolling_resistance_n(p, kV100, 0.0);
    const PowertrainPoint pt = compute_powertrain(p, f, kV100);
    EXPECT_NEAR(pt.motor_elec_w + p.aux.power_w(p.env.ambient_temp_c), 15344.0, 2.0);
}

TEST(Physics, GradeForceIsAntisymmetric) {
    const ModelParams p = simple();
    EXPECT_NEAR(grade_force_n(p, 10.0), 1757.0, 0.5);
    EXPECT_DOUBLE_EQ(grade_force_n(p, -6.0), -grade_force_n(p, 6.0));
}

TEST(Physics, NoRollingResistanceAtStandstillAndNegativeSpeedRejected) {
    const ModelParams p = simple();
    EXPECT_DOUBLE_EQ(rolling_resistance_n(p, 0.0, 0.0), 0.0);
    EXPECT_THROW(aero_drag_n(p, -1.0), std::invalid_argument);
}

TEST(Powertrain, MotoringPowerBalance) {
    const ModelParams p = generic();
    const PowertrainPoint pt = compute_powertrain(p, 800.0, 20.0);
    EXPECT_NEAR(pt.motor_elec_w, 800.0 * 20.0 + pt.gearbox_loss_w + pt.motor_loss_w, 1e-9);
}

TEST(Powertrain, BrakingSplitsIntoRegenAndFriction) {
    const ModelParams p = generic();
    const PowertrainPoint pt = compute_powertrain(p, -2500.0, 20.0);
    EXPECT_NEAR(pt.regen_wheel_w + pt.friction_brake_w, 2500.0 * 20.0, 1e-9);
    EXPECT_NEAR(-pt.motor_elec_w, pt.regen_wheel_w - pt.gearbox_loss_w - pt.motor_loss_w, 1e-9);
    EXPECT_LT(pt.motor_elec_w, 0.0);
}

TEST(Powertrain, TorqueEnvelope) {
    const ModelParams p = generic();
    EXPECT_DOUBLE_EQ(p.motor.max_torque_at(100.0), 310.0);
    EXPECT_DOUBLE_EQ(p.motor.max_torque_at(1000.0), 150.0);
    EXPECT_TRUE(compute_powertrain(p, 12000.0, 10.0).torque_limited);
}
