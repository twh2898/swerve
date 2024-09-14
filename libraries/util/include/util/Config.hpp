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

    struct SimConfig {
        double flip;
        double stop;
        int step;
    };

    struct TelemetryConfig {
        int port;
        string address;
    };

    struct PIDConfig {
        double p;
        double i;
        double d;
    };

    struct SwerveConfig {
        PIDConfig pid;
    };

    struct Config {
        json data;

        SimConfig sim;

        TelemetryConfig telemetry;
        SwerveConfig swerve;

        static Config fromFile(const string & file);
    };
}
