#include "evdynasim/interp.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "evdynasim/config.hpp"

namespace evds {
namespace {

void check_axis(const std::vector<double>& axis, const char* name) {
    if (axis.size() < 2) throw std::invalid_argument(std::string(name) + " needs at least two points");
    for (std::size_t i = 1; i < axis.size(); ++i) {
        if (!(axis[i] > axis[i - 1])) {
            throw std::invalid_argument(std::string(name) + " must be strictly increasing");
        }
    }
}

// Cell index i (axis[i] <= x < axis[i+1]) and weight w, with clamping.
std::pair<std::size_t, double> locate(const std::vector<double>& axis, double x) {
    const double xc = std::clamp(x, axis.front(), axis.back());
    const auto upper = std::upper_bound(axis.begin(), axis.end(), xc);
    std::ptrdiff_t i = (upper - axis.begin()) - 1;
    i = std::clamp<std::ptrdiff_t>(i, 0, static_cast<std::ptrdiff_t>(axis.size()) - 2);
    const auto k = static_cast<std::size_t>(i);
    return {k, (xc - axis[k]) / (axis[k + 1] - axis[k])};
}

}  // namespace

Table1D::Table1D(std::vector<double> x, std::vector<double> y) : x_(std::move(x)), y_(std::move(y)) {
    check_axis(x_, "Table1D x");
    if (x_.size() != y_.size()) throw std::invalid_argument("Table1D x and y must have the same length");
}

double Table1D::operator()(double x) const {
    const auto [i, w] = locate(x_, x);
    return (1.0 - w) * y_[i] + w * y_[i + 1];
}

Table2D::Table2D(std::vector<double> x, std::vector<double> y, std::vector<double> z)
    : x_(std::move(x)), y_(std::move(y)), z_(std::move(z)) {
    check_axis(x_, "Table2D x");
    check_axis(y_, "Table2D y");
    if (z_.size() != x_.size() * y_.size()) throw std::invalid_argument("Table2D z has the wrong size");
}

double Table2D::operator()(double x, double y) const {
    const auto [i, wx] = locate(x_, x);
    const auto [j, wy] = locate(y_, y);
    const std::size_t ny = y_.size();
    const double z00 = z_[i * ny + j];
    const double z10 = z_[(i + 1) * ny + j];
    const double z01 = z_[i * ny + j + 1];
    const double z11 = z_[(i + 1) * ny + j + 1];
    return (1 - wx) * (1 - wy) * z00 + wx * (1 - wy) * z10 + (1 - wx) * wy * z01 + wx * wy * z11;
}

Table2D Table2D::from_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open table file: " + path);
    std::string line;
    std::vector<double> x, y, z;

    std::getline(file, line);  // header: label, y0, y1, ...
    {
        std::istringstream cells(line);
        std::string cell;
        std::getline(cells, cell, ',');  // skip label
        while (std::getline(cells, cell, ',')) y.push_back(parse_double(cell));
    }
    while (std::getline(file, line)) {
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;
        std::istringstream cells(line);
        std::string cell;
        std::getline(cells, cell, ',');
        x.push_back(parse_double(cell));
        std::size_t count = 0;
        while (std::getline(cells, cell, ',')) {
            z.push_back(parse_double(cell));
            ++count;
        }
        if (count != y.size()) throw std::runtime_error(path + ": row with wrong number of columns");
    }
    return Table2D(std::move(x), std::move(y), std::move(z));
}

}  // namespace evds
