// Command line front end of the C++ engine.
//
//   evdynasim_cli --config configs/generic_ev.yaml --cycle nedc
//   evdynasim_cli --config configs/generic_ev.yaml --cycle my_drive.csv --out result.csv
//   evdynasim_cli --config configs/generic_ev.yaml --cycle nedc --repeat 1000   (benchmark)
#include <chrono>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>

#include "evdynasim/simulator.hpp"

namespace {

void usage() {
    std::cerr << "usage: evdynasim_cli --config FILE [--cycle nedc|FILE.csv] [--out FILE.csv] [--repeat N]\n";
}

void print_summary(const evds::Summary& s) {
    const auto row = [](const char* name, double v) { std::printf("  %-28s %12.4f\n", name, v); };
    row("duration_s", s.duration_s);
    row("distance_km", s.distance_km);
    row("battery_energy_kwh", s.battery_energy_kwh);
    row("consumption_kwh_per_100km", s.consumption_kwh_per_100km);
    row("chemical_energy_kwh", s.chemical_energy_kwh);
    row("regen_energy_kwh", s.regen_energy_kwh);
    row("aero_kwh", s.aero_kwh);
    row("rolling_kwh", s.rolling_kwh);
    row("grade_kwh", s.grade_kwh);
    row("gearbox_loss_kwh", s.gearbox_loss_kwh);
    row("motor_loss_kwh", s.motor_loss_kwh);
    row("friction_brake_kwh", s.friction_brake_kwh);
    row("aux_kwh", s.aux_kwh);
    row("battery_loss_kwh", s.battery_loss_kwh);
    row("energy_balance_error_kwh", s.chemical_energy_kwh - s.sink_total_kwh());
    row("peak_battery_power_kw", s.peak_battery_power_kw);
    row("final_soc", s.final_soc);
    row("max_temp_c", s.max_temp_c);
    std::printf("  %-28s %12d\n", "torque_limited_steps", s.torque_limited_steps);
    std::printf("  %-28s %12d\n", "battery_limited_steps", s.battery_limited_steps);
}

void write_csv(const std::string& path, const evds::SimulationResult& r) {
    std::ofstream out(path);
    out << "t_s,speed_kmh,accel_mps2,p_wheel_kw,motor_speed_rpm,motor_torque_nm,p_battery_kw,current_a,"
           "terminal_v,soc_end,temp_end_c\n";
    out.precision(10);
    for (std::size_t k = 0; k < r.t_start_s.size(); ++k) {
        out << r.t_start_s[k] << ',' << r.speed_mps[k] * 3.6 << ',' << r.accel_mps2[k] << ','
            << r.wheel_power_w[k] / 1e3 << ',' << r.motor_speed_rads[k] * evds::kRadsToRpm << ','
            << r.motor_torque_nm[k] << ',' << r.battery_power_w[k] / 1e3 << ',' << r.current_a[k] << ','
            << r.terminal_v[k] << ',' << r.soc[k + 1] << ',' << r.temp_c[k + 1] << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string config, cycle_arg = "nedc", out;
    int repeat = 1;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (i + 1 >= argc) { usage(); return 2; }
        if (arg == "--config") config = argv[++i];
        else if (arg == "--cycle") cycle_arg = argv[++i];
        else if (arg == "--out") out = argv[++i];
        else if (arg == "--repeat") repeat = std::stoi(argv[++i]);
        else { usage(); return 2; }
    }
    if (config.empty() || repeat < 1) { usage(); return 2; }

    try {
        const evds::ModelParams params = evds::load_params(config);
        const evds::DriveCycle cycle = cycle_arg == "nedc" ? evds::nedc() : evds::load_cycle_csv(cycle_arg);

        const auto start = std::chrono::steady_clock::now();
        evds::SimulationResult result;
        for (int i = 0; i < repeat; ++i) result = evds::simulate(params, cycle);
        const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;

        std::cout << "Vehicle : " << params.name << "\nCycle   : " << cycle.name << "\n";
        print_summary(result.summary);
        std::printf("  %-28s %12.4f\n", "runtime_ms_per_run", elapsed.count() / repeat);
        if (!out.empty()) {
            write_csv(out, result);
            std::cout << "time series written to " << out << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
