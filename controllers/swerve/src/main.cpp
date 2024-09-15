#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
using namespace webots;

#include <stdexcept>

#include "Platform.hpp"
#include "util/Config.hpp"
#include "util/Profiler.hpp"
#include "util/Telemetry.hpp"
#include "util/log.hpp"

using namespace util;
using namespace swerve;

int main() {
    Logging::init_logging(spdlog::level::trace);
    Logging::Main->debug("Logging enabled");

    Config config;
    try {
        config = Config::fromFile("config.json");
    }
    catch (ConfigLoadException & e) {
        auto what = e.what();
        Logging::Main->error("Failed to load config: {}", what);
        return EXIT_FAILURE;
    }

    Telemetry tel(config.telemetry.port, config.telemetry.address);

    Platform::Ptr platform = make_shared<Platform>();
    int time_step = config.sim.step;
    if (time_step < platform->robot.getBasicTimeStep())
        time_step = platform->robot.getBasicTimeStep();
    platform->enable(time_step);

    int wheelSpeed, axisTarget;

    Logging::Main->info("Initialization complete");

    Profiler prof;

    R_DEF_CLOCK(prof, clkMain, "main");
    R_DEF_CLOCK(prof, clkSim, "simulation");
    R_DEF_CLOCK(prof, clkPlan, "planner");
    R_DEF_CLOCK(prof, clkTelem, "telemetry");

    // LED led("led1");
    // led.set(0);

    clkSim->reset();
    while (platform->step(time_step) != -1) {
        if (config.sim.stop > 0 && platform->robot.getTime() >= config.sim.stop)
            break;

        clkSim->tick();

        if (platform->robot.getTime() >= config.sim.flip) {
            // axisTarget = 1;
            wheelSpeed = 5;
            // led.set(1);
        }
        else {
            // axisSpeed = SPEED / 5;
            // axisTarget = 2;
            axisTarget = 1;
            wheelSpeed = 0;
        }

        platform->rightDrive->setSteer(axisTarget);
        platform->leftDrive->setSteer(axisTarget);

        platform->rightDrive->setDriveVelocity(wheelSpeed);
        platform->leftDrive->setDriveVelocity(wheelSpeed);

        R_PROFILE_STEP(clkTelem, {
            tel.send(platform.get());
            // tel.send(&planner);
        });

        clkMain->tick();

        json profile {
            {"profile", prof.getTelemetry()},
        };
        tel.send(profile);

        clkSim->reset();
    }

    return 0;
}
