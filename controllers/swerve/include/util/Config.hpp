#pragma once

#include <exception>
#include <string>

#include "json.hpp"

namespace util {
    using json = nlohmann::json;

    using std::string;
    using std::map;

    class ConfigLoadException : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    struct SimConfig {
        double flip;
        double stop;
        int step;
    };

    struct TelemetryConfig {
        int port;
        string address;
    };

    struct ControllerConfig {
        double driveAccel;
        double steerAccel;
    };

    struct Config {
        json data;

        SimConfig sim;

        ControllerConfig controller;

        TelemetryConfig telemetry;

        static Config fromFile(const string & file);
    };
}
