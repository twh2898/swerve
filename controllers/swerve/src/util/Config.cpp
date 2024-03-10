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

        if (!data.contains("sim"))
            throw ConfigLoadException("Missing key: sim");
        auto simConfig = data["sim"];

        if (!simConfig.contains("flip"))
            throw ConfigLoadException("Missing key: sim.flip");

        if (!simConfig.contains("stop"))
            throw ConfigLoadException("Missing key: sim.stop");

        if (!simConfig.contains("step"))
            throw ConfigLoadException("Missing key: sim.step");

        SimConfig sim {
            flip: simConfig["flip"],
            stop: simConfig["stop"],
            step: simConfig["step"],
        };

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

        if (!data.contains("swerve"))
            throw ConfigLoadException("Missing key: swerve");
        auto swerveConfig = data["swerve"];

        if (!swerveConfig.contains("pid"))
            throw ConfigLoadException("Missing key: swerve.pid");
        auto pidConfig = swerveConfig["pid"];

        if (!pidConfig.contains("p"))
            throw ConfigLoadException("Missing key: swerve.pid.p");

        if (!pidConfig.contains("i"))
            throw ConfigLoadException("Missing key: swerve.pid.i");

        if (!pidConfig.contains("d"))
            throw ConfigLoadException("Missing key: swerve.pid.d");

        PIDConfig pid {
            p : pidConfig["p"],
            i : pidConfig["i"],
            d : pidConfig["d"],
        };

        SwerveConfig swerve {
            pid : pid,
        };

        return Config {
            data : data,
            sim: sim,
            telemetry : telem,
            swerve : swerve,
        };
    }
}
