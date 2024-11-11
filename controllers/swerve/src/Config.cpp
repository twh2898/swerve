#include "util/Config.hpp"

#include <fstream>

#include "util/log.hpp"

#define TEST_KEY(data, key)    \
    if (!(data).contains(key)) \
        throw ConfigLoadException("Missing key: " key);

#define TEST_SUB_KEY(data, prefix, key) \
    if (!(data).contains(key))          \
        throw ConfigLoadException("Missing key: " prefix "." key);


namespace util {
    using std::fstream;

    Config Config::fromFile(const string & file) {
        Logging::Config->info("Loading config from {}", file);
        fstream f(file);
        json data = json::parse(f);

        auto ds = data.dump();
        Logging::Config->debug("Config data is {}", ds);

        TEST_KEY(data, "sim")
        auto simConfig = data["sim"];

        TEST_SUB_KEY(simConfig, "sim", "flip")
        TEST_SUB_KEY(simConfig, "sim", "stop")
        TEST_SUB_KEY(simConfig, "sim", "step")

        SimConfig sim {
            flip : simConfig["flip"],
            stop : simConfig["stop"],
            step : simConfig["step"],
        };

        TEST_KEY(data, "controller")
        auto controllerConfig = data["controller"];

        TEST_SUB_KEY(controllerConfig, "controller", "accel")

        ControllerConfig controller {
            accel : controllerConfig["accel"],
        };

        TEST_KEY(data, "telemetry")
        auto telemConfig = data["telemetry"];

        TEST_SUB_KEY(telemConfig, "telemetry", "port")
        TEST_SUB_KEY(telemConfig, "telemetry", "address")

        TelemetryConfig telem {
            port : telemConfig["port"],
            address : telemConfig["address"],
        };

        return Config {
            data : data,
            sim : sim,
            controller : controller,
            telemetry : telem,
        };
    }
}
