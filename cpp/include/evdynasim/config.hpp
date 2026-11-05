// Reader for the simple YAML subset used by the vehicle configuration files.
//
// Supported: "section:" lines, indented "key: value" lines, top-level
// "key: value", numbers, quoted strings, one-line lists "[1, 2, 3]",
// comments starting with '#'. Keys are stored flat as "section.key".
//
// Design decision: a full YAML library (yaml-cpp) would be the usual choice,
// but the configuration format is deliberately tiny, and avoiding the
// dependency keeps the engine buildable with nothing but a compiler and
// CMake. The Python side uses PyYAML on the same files; the tests make sure
// both readers agree.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace evds {

class Config {
public:
    static Config load(const std::string& path);
    static Config parse(const std::string& text, const std::string& source = "<string>");

    bool has(const std::string& key) const;
    double number(const std::string& key) const;
    std::string text(const std::string& key, const std::string& fallback = "") const;
    std::vector<double> list(const std::string& key) const;

    // Folder of the file the configuration was loaded from (for relative paths).
    const std::string& base_dir() const { return base_dir_; }

private:
    const std::string& raw(const std::string& key) const;
    std::map<std::string, std::string> values_;
    std::string source_;
    std::string base_dir_;
};

// Locale-independent string -> double. std::stod would read "1.5" wrongly on
// a machine with a German locale (decimal comma).
double parse_double(const std::string& s);

}  // namespace evds
