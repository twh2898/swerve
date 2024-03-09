#include "util/Config.hpp"

#include <fstream>

#include "util/log.hpp"

namespace util {
    using std::fstream;

    Config Config::fromFile(const string & file) {
        Logging::Config->info("Loading config from {}", file);
        fstream f(file);
        json data = json::parse(f);

        auto ds = data.dump();
        Logging::Config->debug("Config data is {}", ds);

        if (!data.contains("telemetry"))
            throw ConfigLoadException("Missing key: telemetry");
        auto telemConfig = data["telemetry"];

        if (!telemConfig.contains("port"))
            throw ConfigLoadException("Missing key: telemetry.port");

        if (!telemConfig.contains("address"))
            throw ConfigLoadException("Missing key: telemetry.address");

        TelemetryConfig telem {
            port : telemConfig["port"],
            address : telemConfig["address"],
        };

        if (!data.contains("pid"))
            throw ConfigLoadException("Missing key: pid");
        auto pidConfig = data["pid"];

        if (!pidConfig.contains("p"))
            throw ConfigLoadException("Missing key: pid.p");

        if (!pidConfig.contains("i"))
            throw ConfigLoadException("Missing key: pid.i");

        if (!pidConfig.contains("d"))
            throw ConfigLoadException("Missing key: pid.d");

        if (!pidConfig.contains("speed"))
            throw ConfigLoadException("Missing key: pid.speed");

        PIDConfig pid {
            p : pidConfig["p"],
            i : pidConfig["i"],
            d : pidConfig["d"],
            speed : pidConfig["speed"],
        };

        if (!data.contains("tuneMode"))
            throw ConfigLoadException("Missing key: tuneMode");
        bool tuneMode = data["tuneMode"];

        if (!data.contains("target"))
            throw ConfigLoadException("Missing key: target");
        auto targetConfig = data["target"];

        if (!targetConfig.contains("heading"))
            throw ConfigLoadException("Missing key: target.heading");

        if (!targetConfig.contains("size"))
            throw ConfigLoadException("Missing key: target.size");

        TargetConfig target {
            heading : targetConfig["heading"],
            size : targetConfig["size"],
        };

        return Config {
            data : data,
            telemetry : telem,
            pid : pid,
            tuneMode : tuneMode,
            target : target,
        };
    }
}
