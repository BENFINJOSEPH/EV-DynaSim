#include "evdynasim/config.hpp"

#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace evds {
namespace {

std::string trim(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

std::string strip_comment(const std::string& line) {
    bool in_quotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '"') in_quotes = !in_quotes;
        if (line[i] == '#' && !in_quotes) return line.substr(0, i);
    }
    return line;
}

std::string unquote(const std::string& s) {
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"') return s.substr(1, s.size() - 2);
    return s;
}

}  // namespace

double parse_double(const std::string& s) {
    std::istringstream in(trim(s));
    in.imbue(std::locale::classic());
    double value = 0.0;
    in >> value;
    if (in.fail() || !(in >> std::ws).eof()) {
        throw std::invalid_argument("not a number: '" + s + "'");
    }
    return value;
}

Config Config::load(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open configuration file: " + path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    Config cfg = parse(buffer.str(), path);
    const auto slash = path.find_last_of("/\\");
    cfg.base_dir_ = (slash == std::string::npos) ? "." : path.substr(0, slash);
    return cfg;
}

Config Config::parse(const std::string& text, const std::string& source) {
    Config cfg;
    cfg.source_ = source;
    cfg.base_dir_ = ".";
    std::istringstream lines(text);
    std::string line;
    std::string section;
    int line_no = 0;
    while (std::getline(lines, line)) {
        ++line_no;
        const std::string content = strip_comment(line);
        const std::string body = trim(content);
        if (body.empty()) continue;

        const bool indented = content.find_first_not_of(" \t") > 0;
        const auto colon = body.find(':');
        if (colon == std::string::npos) {
            throw std::runtime_error(source + ":" + std::to_string(line_no) + ": expected 'key: value'");
        }
        const std::string key = trim(body.substr(0, colon));
        const std::string value = trim(body.substr(colon + 1));

        if (!indented && value.empty()) {
            section = key;  // start of a new section
        } else if (!indented) {
            section.clear();
            cfg.values_[key] = value;
        } else {
            if (section.empty()) {
                throw std::runtime_error(source + ":" + std::to_string(line_no) +
                                         ": indented key outside a section");
            }
            cfg.values_[section + "." + key] = value;
        }
    }
    return cfg;
}

bool Config::has(const std::string& key) const { return values_.count(key) > 0; }

const std::string& Config::raw(const std::string& key) const {
    const auto it = values_.find(key);
    if (it == values_.end()) throw std::runtime_error(source_ + ": missing configuration key '" + key + "'");
    return it->second;
}

double Config::number(const std::string& key) const {
    try {
        return parse_double(raw(key));
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error(source_ + ": key '" + key + "': " + e.what());
    }
}

std::string Config::text(const std::string& key, const std::string& fallback) const {
    return has(key) ? unquote(raw(key)) : fallback;
}

std::vector<double> Config::list(const std::string& key) const {
    const std::string& s = raw(key);
    if (s.size() < 2 || s.front() != '[' || s.back() != ']') {
        throw std::runtime_error(source_ + ": key '" + key + "' is not a [list]");
    }
    std::vector<double> out;
    std::istringstream items(s.substr(1, s.size() - 2));
    std::string item;
    while (std::getline(items, item, ',')) {
        if (!trim(item).empty()) out.push_back(parse_double(item));
    }
    return out;
}

}  // namespace evds
