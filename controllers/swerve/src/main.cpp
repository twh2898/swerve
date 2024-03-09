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

#define SPEED 5
#define TIME_STEP 64

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

    int movementCounter = 10;

    Platform::Ptr platform = make_shared<Platform>();
    platform->enable(TIME_STEP);

    int wheelSpeed, axisSpeed;

    Logging::Main->info("Initialization complete");

    Profiler prof;

    R_DEF_CLOCK(prof, clkMain, "main");
    R_DEF_CLOCK(prof, clkSim, "simulation");
    R_DEF_CLOCK(prof, clkPlan, "planner");
    R_DEF_CLOCK(prof, clkTelem, "telemetry");

    clkSim->reset();
    while (platform->robot.step(TIME_STEP) != -1) {
        clkSim->tick();

        if (movementCounter == 0) {
            // axisSpeed = 0;
            wheelSpeed = SPEED;
        }
        else {
            axisSpeed = SPEED / 5;
            wheelSpeed = 0;
            movementCounter--;
        }

        platform->frontRight->axisMotor->setVelocity(axisSpeed);
        platform->frontLeft->axisMotor->setVelocity(axisSpeed);
        platform->backRight->axisMotor->setVelocity(axisSpeed);
        platform->backLeft->axisMotor->setVelocity(axisSpeed);

        platform->frontRight->wheelMotor->setVelocity(wheelSpeed);
        platform->frontLeft->wheelMotor->setVelocity(wheelSpeed);
        platform->backRight->wheelMotor->setVelocity(wheelSpeed);
        platform->backLeft->wheelMotor->setVelocity(wheelSpeed);

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
