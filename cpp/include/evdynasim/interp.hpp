// Lookup tables with linear (1-D) and bilinear (2-D) interpolation.
// Same algorithm as src/evdynasim/interp.py: identical index search and
// clamping at the grid edges, so both implementations give the same numbers.
#pragma once

#include <string>
#include <vector>

namespace evds {

class Table1D {
public:
    Table1D() = default;
    Table1D(std::vector<double> x, std::vector<double> y);
    double operator()(double x) const;
    const std::vector<double>& x() const { return x_; }
    const std::vector<double>& y() const { return y_; }

private:
    std::vector<double> x_, y_;
};

class Table2D {
public:
    Table2D() = default;
    // z is row-major: z[i * ny + j] = f(x[i], y[j])
    Table2D(std::vector<double> x, std::vector<double> y, std::vector<double> z);
    double operator()(double x, double y) const;
    static Table2D from_csv(const std::string& path);

private:
    std::vector<double> x_, y_, z_;
};

}  // namespace evds
