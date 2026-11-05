#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "evdynasim/simulator.hpp"

using namespace evds;

namespace {
ModelParams load(const char* name) { return load_params(std::string(EVDS_REPO_DIR) + "/configs/" + name); }

DriveCycle hill_cycle() {
    DriveCycle c{"hill", {}, {}, {}};
    for (int k = 0; k <= 300; ++k) {
        const double t = k;
        const double v = t < 30 ? 25.0 * t / 30 : (t < 270 ? 25.0 : 25.0 * (300 - t) / 30);
        double g = 0;
        if (t < 60) g = 6 * t / 60;
        else if (t < 120) g = 6;
        else if (t < 180) g = 6 - 12 * (t - 120) / 60;
        else if (t < 240) g = -6;
        else g = -6 + 6 * (t - 240) / 60;
        c.time_s.push_back(t);
        c.speed_mps.push_back(v);
        c.grade_pct.push_back(g);
    }
    return c;
}
}  // namespace

TEST(Simulator, NedcDurationAndDistance) {
    const DriveCycle c = nedc();
    EXPECT_DOUBLE_EQ(c.duration_s(), 1180.0);
    const SimulationResult r = simulate(load("generic_ev.yaml"), c);
    EXPECT_NEAR(r.summary.distance_km, 11.007, 0.002 * 11.007);
}

TEST(Simulator, EnergyBalanceClosesOnNedcAndHill) {
    for (const char* cfg : {"simple_ev.yaml", "generic_ev.yaml"}) {
        for (const DriveCycle& c : {nedc(), hill_cycle()}) {
            const Summary s = simulate(load(cfg), c).summary;
            EXPECT_NEAR(s.sink_total_kwh(), s.chemical_energy_kwh, 1e-10 * s.chemical_energy_kwh);
        }
    }
}

TEST(Simulator, SimpleModelReproducesV02Result) {
    const Summary s = simulate(load("simple_ev.yaml"), nedc()).summary;
    EXPECT_NEAR(s.battery_energy_kwh, 1.4558, 1e-4);
}

TEST(Simulator, ConstantSpeedConsumption) {
    const Summary s = simulate(load("simple_ev.yaml"), constant_speed(100.0, 3600.0)).summary;
    EXPECT_NEAR(s.distance_km, 100.0, 1e-9);
    EXPECT_NEAR(s.consumption_kwh_per_100km, 15.344, 0.002);
}

TEST(Simulator, RegenCurtailedWhenBatteryIsFull) {
    ModelParams p = load("generic_ev.yaml");
    p.battery.initial_soc = 0.995;
    const Summary s = simulate(p, hill_cycle()).summary;
    EXPECT_GT(s.regen_curtailed_kwh, 0.0);
    EXPECT_NEAR(s.sink_total_kwh(), s.chemical_energy_kwh, 1e-10 * s.chemical_energy_kwh);
}

TEST(Simulator, InvalidCycleRejected) {
    DriveCycle c{"bad", {0.0, 1.0, 1.0}, {0.0, 1.0, 2.0}, {0.0, 0.0, 0.0}};
    EXPECT_THROW(simulate(load("simple_ev.yaml"), c), std::invalid_argument);
}
