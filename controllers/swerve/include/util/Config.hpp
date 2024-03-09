#pragma once

#include <exception>
#include <string>

#include "PID.hpp"
#include "json.hpp"

namespace util {
    using json = nlohmann::json;

    using std::string;
    using std::map;

    class ConfigLoadException : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    struct TelemetryConfig {
        int port;
        string address;
    };

    struct PIDConfig {
        double p;
        double i;
        double d;
        double speed;
    };

    struct TargetConfig {
        double heading;
        double size;
    };

    struct Config {
        json data;

        TelemetryConfig telemetry;
        PIDConfig pid;
        bool tuneMode;
        TargetConfig target;

        static Config fromFile(const string & file);
    };
}
