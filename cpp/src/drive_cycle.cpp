#include "evdynasim/drive_cycle.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "evdynasim/config.hpp"
#include "evdynasim/params.hpp"

namespace evds {
namespace {

// Source: UNECE Regulation No. 83, Annex 4. Same tables as cycles.py.
const std::vector<std::pair<double, double>> kEce15 = {
    {0, 0},    {11, 0},   {15, 15},  {23, 15},  {25, 10},  {28, 0},   {49, 0},   {54, 15},  {56, 15},
    {61, 32},  {85, 32},  {93, 10},  {96, 0},   {117, 0},  {122, 15}, {124, 15}, {133, 35}, {135, 35},
    {143, 50}, {155, 50}, {163, 35}, {176, 35}, {178, 32}, {185, 10}, {188, 0},  {195, 0}};
const std::vector<std::pair<double, double>> kEudc = {
    {0, 0},     {20, 0},    {25, 15},   {27, 15},   {36, 35},   {38, 35},   {46, 50},   {48, 50},
    {61, 70},   {111, 70},  {119, 50},  {188, 50},  {201, 70},  {251, 70},  {286, 100}, {316, 100},
    {336, 120}, {346, 120}, {362, 80},  {370, 50},  {380, 0},   {400, 0}};

double interp_breakpoints(const std::vector<std::pair<double, double>>& pts, double t) {
    if (t <= pts.front().first) return pts.front().second;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        if (t <= pts[i].first) {
            const auto [t0, v0] = pts[i - 1];
            const auto [t1, v1] = pts[i];
            return v0 + (v1 - v0) * (t - t0) / (t1 - t0);
        }
    }
    return pts.back().second;
}

}  // namespace

void DriveCycle::validate() const {
    const std::size_t n = time_s.size();
    if (n < 2) throw std::invalid_argument("a drive cycle needs at least two samples");
    if (speed_mps.size() != n || grade_pct.size() != n) {
        throw std::invalid_argument("time, speed and grade must have the same length");
    }
    for (std::size_t k = 0; k < n; ++k) {
        if (k > 0 && !(time_s[k] > time_s[k - 1])) throw std::invalid_argument("time must be strictly increasing");
        if (!std::isfinite(speed_mps[k]) || !std::isfinite(grade_pct[k])) {
            throw std::invalid_argument("cycle contains NaN or infinite values");
        }
        if (speed_mps[k] < 0.0) throw std::invalid_argument("speed must not be negative");
    }
}

DriveCycle nedc(double dt) {
    std::vector<std::pair<double, double>> pts;
    double offset = 0.0;
    for (int seg = 0; seg < 5; ++seg) {
        const auto& table = seg < 4 ? kEce15 : kEudc;
        for (std::size_t i = pts.empty() ? 0 : 1; i < table.size(); ++i) {
            pts.emplace_back(table[i].first + offset, table[i].second);
        }
        offset += table.back().first;
    }
    DriveCycle c{"NEDC", {}, {}, {}};
    const double t_end = pts.back().first;
    for (std::size_t k = 0;; ++k) {
        const double t = static_cast<double>(k) * dt;
        if (t > t_end + 0.5 * dt) break;
        c.time_s.push_back(t);
        c.speed_mps.push_back(interp_breakpoints(pts, t) * kKmhToMps);
        c.grade_pct.push_back(0.0);
    }
    return c;
}

DriveCycle constant_speed(double speed_kmh, double duration_s, double dt) {
    DriveCycle c{"constant", {}, {}, {}};
    for (std::size_t k = 0;; ++k) {
        const double t = static_cast<double>(k) * dt;
        if (t > duration_s + 0.5 * dt) break;
        c.time_s.push_back(t);
        c.speed_mps.push_back(speed_kmh * kKmhToMps);
        c.grade_pct.push_back(0.0);
    }
    return c;
}

DriveCycle load_cycle_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open cycle file: " + path);
    std::string line;
    std::getline(file, line);
    int col_t = -1, col_v = -1, col_g = -1, ncols = 0;
    {
        std::istringstream cells(line);
        std::string cell;
        while (std::getline(cells, cell, ',')) {
            cell.erase(cell.find_last_not_of(" \r\n\t") + 1);
            if (cell == "time_s") col_t = ncols;
            if (cell == "speed_kmh") col_v = ncols;
            if (cell == "grade_pct") col_g = ncols;
            ++ncols;
        }
    }
    if (col_t < 0 || col_v < 0) throw std::runtime_error(path + ": needs columns time_s and speed_kmh");

    std::string stem = path.substr(path.find_last_of("/\\") + 1);
    stem = stem.substr(0, stem.find_last_of('.'));
    DriveCycle c{stem, {}, {}, {}};
    while (std::getline(file, line)) {
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;
        std::vector<double> row;
        std::istringstream cells(line);
        std::string cell;
        while (std::getline(cells, cell, ',')) row.push_back(parse_double(cell));
        if (static_cast<int>(row.size()) != ncols) throw std::runtime_error(path + ": wrong number of columns");
        c.time_s.push_back(row[col_t]);
        c.speed_mps.push_back(row[col_v] * kKmhToMps);
        c.grade_pct.push_back(col_g >= 0 ? row[col_g] : 0.0);
    }
    c.validate();
    return c;
}

}  // namespace evds
