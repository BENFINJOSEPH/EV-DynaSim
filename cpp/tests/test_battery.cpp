#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "evdynasim/battery.hpp"

using namespace evds;

namespace {
ModelParams generic() { return load_params(std::string(EVDS_REPO_DIR) + "/configs/generic_ev.yaml"); }
}  // namespace

TEST(Battery, CurrentForPowerSolvesThePowerEquation) {
    const CurrentSolution ideal = current_for_power(400.0, 0.0, 40000.0);
    EXPECT_DOUBLE_EQ(ideal.current_a, 100.0);
    const CurrentSolution s = current_for_power(380.0, 0.05, 50000.0);
    EXPECT_NEAR((380.0 - 0.05 * s.current_a) * s.current_a, 50000.0, 1e-6);
    EXPECT_FALSE(s.beyond_max_power);
    EXPECT_TRUE(current_for_power(380.0, 0.05, 800000.0).beyond_max_power);
}

TEST(Battery, ResistanceRisesInTheCold) {
    const ModelParams p = generic();
    EXPECT_NEAR(p.battery.resistance_factor(-10.0), std::exp(3000.0 * (1 / 263.15 - 1 / 298.15)), 1e-12);
    EXPECT_NEAR(p.battery.r0_pack_ohm(25.0), 0.001 * 96 / 4, 1e-15);
}

TEST(Battery, ChargeLimitDeratesNearFull) {
    const ModelParams p = generic();
    EXPECT_DOUBLE_EQ(p.battery.charge_limit_w(0.5), p.battery.max_charge_power_w);
    EXPECT_NEAR(p.battery.charge_limit_w(0.95), 0.5 * p.battery.max_charge_power_w, 1e-6);
    EXPECT_DOUBLE_EQ(p.battery.charge_limit_w(1.0), 0.0);
}

TEST(Battery, EnergyIdentityPerStep) {
    // OCV * I = P_terminal + losses, exactly, for every step
    const ModelParams p = generic();
    BatteryState s = initial_battery_state(p);
    for (int k = 0; k < 100; ++k) {
        const double power = (k % 2 == 0) ? 50000.0 : -20000.0;
        const BatteryStepInfo info = battery_step(p, s, power, 1.0);
        EXPECT_NEAR(info.ocv_v * info.current_a, info.terminal_v * info.current_a + info.loss_w, 1e-6);
        EXPECT_NEAR(info.terminal_v * info.current_a, power, 1e-6);
    }
}

TEST(Battery, HeatsUpUnderLoad) {
    const ModelParams p = generic();
    BatteryState s = initial_battery_state(p);
    for (int k = 0; k < 600; ++k) battery_step(p, s, 60000.0, 1.0);
    EXPECT_GT(s.temp_c, p.thermal.initial_temp_c + 0.1);
    EXPECT_LT(s.soc, p.battery.initial_soc);
}
