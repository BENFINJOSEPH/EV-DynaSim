#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "evdynasim/config.hpp"
#include "evdynasim/interp.hpp"
#include "evdynasim/params.hpp"

using namespace evds;

TEST(Config, ParsesSectionsNumbersStringsAndLists) {
    const Config c = Config::parse(
        "name: \"My car # 1\"   # comment\n"
        "vehicle:\n"
        "  mass_kg: 1800.5\n"
        "  list: [1, 2.5, -3]\n");
    EXPECT_EQ(c.text("name"), "My car # 1");
    EXPECT_DOUBLE_EQ(c.number("vehicle.mass_kg"), 1800.5);
    const auto v = c.list("vehicle.list");
    ASSERT_TRUE(v.size() == 3);
    EXPECT_DOUBLE_EQ(v[2], -3.0);
}

TEST(Config, MissingKeyAndBadNumberThrow) {
    const Config c = Config::parse("a:\n  b: hello\n");
    EXPECT_THROW(c.number("a.missing"), std::runtime_error);
    EXPECT_THROW(c.number("a.b"), std::runtime_error);
}

TEST(Config, ParseDoubleIgnoresLocale) {
    EXPECT_DOUBLE_EQ(parse_double(" 1.25 "), 1.25);
    EXPECT_THROW(parse_double("1,25"), std::invalid_argument);
}

TEST(Interp, Table1DInterpolatesAndClamps) {
    const Table1D t({0.0, 1.0, 3.0}, {10.0, 20.0, 0.0});
    EXPECT_DOUBLE_EQ(t(0.5), 15.0);
    EXPECT_DOUBLE_EQ(t(2.0), 10.0);
    EXPECT_DOUBLE_EQ(t(-5.0), 10.0);
    EXPECT_DOUBLE_EQ(t(9.0), 0.0);
}

TEST(Interp, Table2DIsExactForBilinearFunctions) {
    const std::vector<double> x{0.0, 1.0, 2.0}, y{0.0, 10.0};
    std::vector<double> z;
    for (double xi : x)
        for (double yj : y) z.push_back(1 + 2 * xi + 0.3 * yj + 0.05 * xi * yj);
    const Table2D t(x, y, z);
    EXPECT_NEAR(t(1.7, 9.0), 1 + 2 * 1.7 + 0.3 * 9.0 + 0.05 * 1.7 * 9.0, 1e-12);
}

TEST(Interp, UnsortedAxisRejected) {
    EXPECT_THROW(Table1D({0.0, 2.0, 1.0}, {1.0, 2.0, 3.0}), std::invalid_argument);
}

TEST(Params, LoadsGenericConfigWithLossMap) {
    const ModelParams p = load_params(std::string(EVDS_REPO_DIR) + "/configs/generic_ev.yaml");
    EXPECT_DOUBLE_EQ(p.body.mass_kg, 1800.0);
    EXPECT_DOUBLE_EQ(p.motor.max_power_w, 150000.0);
    EXPECT_TRUE(p.motor.loss_map.has_value());
    EXPECT_DOUBLE_EQ(p.battery.capacity_ah(), 200.0);
}

TEST(Params, InvalidValueRejected) {
    ModelParams p = load_params(std::string(EVDS_REPO_DIR) + "/configs/simple_ev.yaml");
    p.drivetrain.gearbox_efficiency = 1.2;
    EXPECT_THROW(p.validate(), std::invalid_argument);
}
