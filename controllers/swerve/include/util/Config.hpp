#pragma once

#include <exception>
#include <optional>
#include <string>
#include <vector>

#include "json.hpp"

namespace util {
    using json = nlohmann::json;

    using std::string;
    using std::vector;
    using std::optional;

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

    struct Mission {
        optional<string> name;
        double power;
        double direction;
        double spin;
        double duration;
    };

    struct Config {
        json data;

        SimConfig sim;

        ControllerConfig controller;

        TelemetryConfig telemetry;

        vector<Mission> mission;

        static Config fromFile(const string & file);
    };
}
