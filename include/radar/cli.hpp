#pragma once

#include <cstdlib>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace radar::cli {

/// Minimal "--key value" / "--flag" command-line parser shared by the apps.
class Args {
public:
    Args(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string key = argv[i];
            if (key.rfind("--", 0) != 0) {
                throw std::invalid_argument("unexpected argument: " + key);
            }
            key = key.substr(2);
            if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
                values_[key] = argv[++i];
            } else {
                values_[key] = "";
            }
        }
    }

    bool has(const std::string& key) const { return values_.count(key) != 0; }

    std::string get(const std::string& key, const std::string& fallback) const {
        const auto it = values_.find(key);
        return it == values_.end() ? fallback : it->second;
    }

    double get(const std::string& key, double fallback) const {
        const auto it = values_.find(key);
        return it == values_.end() ? fallback : std::stod(it->second);
    }

    long get(const std::string& key, long fallback) const {
        const auto it = values_.find(key);
        return it == values_.end() ? fallback : std::stol(it->second);
    }

private:
    std::map<std::string, std::string> values_;
};

}  // namespace radar::cli
