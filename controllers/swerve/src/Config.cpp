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

        TEST_SUB_KEY(controllerConfig, "controller", "driveAccel")
        TEST_SUB_KEY(controllerConfig, "controller", "steerAccel")

        ControllerConfig controller {
            driveAccel : controllerConfig["driveAccel"],
            steerAccel : controllerConfig["steerAccel"],
        };

        TEST_KEY(data, "telemetry")
        auto telemConfig = data["telemetry"];

        TEST_SUB_KEY(telemConfig, "telemetry", "port")
        TEST_SUB_KEY(telemConfig, "telemetry", "address")

        TelemetryConfig telem {
            port : telemConfig["port"],
            address : telemConfig["address"],
        };

        TEST_KEY(data, "mission")
        string missionName = data["mission"];

        TEST_KEY(data, "missions")
        auto allMissions = data["missions"];

        if (!allMissions.contains(missionName)) {
            throw ConfigLoadException("Mission name does not exist");
        }

        auto missionConfig = allMissions[missionName];

        vector<Mission> mission;
        for (auto & step : missionConfig) {
            TEST_KEY(step, "power")
            TEST_KEY(step, "direction")
            TEST_KEY(step, "spin")
            TEST_KEY(step, "duration")
            mission.push_back({
                power : step["power"],
                direction : step["direction"],
                spin : step["spin"],
                duration : step["duration"],
            });
        }

        return Config {
            data : data,
            sim : sim,
            controller : controller,
            telemetry : telem,
            mission : mission,
        };
    }
}
