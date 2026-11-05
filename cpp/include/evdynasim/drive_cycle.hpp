// Drive cycle: time, speed and grade samples (SI units, grade in percent).
#pragma once

#include <string>
#include <vector>

namespace evds {

struct DriveCycle {
    std::string name;
    std::vector<double> time_s;
    std::vector<double> speed_mps;
    std::vector<double> grade_pct;

    void validate() const;  // throws std::invalid_argument
    double duration_s() const { return time_s.back() - time_s.front(); }
};

DriveCycle nedc(double dt = 1.0);
DriveCycle constant_speed(double speed_kmh, double duration_s, double dt = 1.0);
// CSV with header containing time_s, speed_kmh and optionally grade_pct (any order).
DriveCycle load_cycle_csv(const std::string& path);

}  // namespace evds
