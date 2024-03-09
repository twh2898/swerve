#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
using namespace webots;

#include <stdexcept>

#include "util/Config.hpp"
#include "util/Profiler.hpp"
#include "util/Telemetry.hpp"
#include "util/log.hpp"

using namespace util;

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
    int wheelSpeed, axisSpeed;

    Robot robot;

    auto fra = robot.getMotor("front right drive axis motor");
    auto fla = robot.getMotor("front left drive axis motor");
    auto bra = robot.getMotor("back right drive axis motor");
    auto bla = robot.getMotor("back left drive axis motor");

    auto frw = robot.getMotor("front right drive wheel motor");
    auto flw = robot.getMotor("front left drive wheel motor");
    auto brw = robot.getMotor("back right drive wheel motor");
    auto blw = robot.getMotor("back left drive wheel motor");

    fra->setPosition(INFINITY);
    fla->setPosition(INFINITY);
    bra->setPosition(INFINITY);
    bla->setPosition(INFINITY);
    frw->setPosition(INFINITY);
    flw->setPosition(INFINITY);
    brw->setPosition(INFINITY);
    blw->setPosition(INFINITY);

    fra->setVelocity(0.0);
    fla->setVelocity(0.0);
    bra->setVelocity(0.0);
    bla->setVelocity(0.0);

    frw->setVelocity(0.0);
    flw->setVelocity(0.0);
    brw->setVelocity(0.0);
    blw->setVelocity(0.0);

    Logging::Main->info("Initialization complete");

    Profiler prof;

    R_DEF_CLOCK(prof, clkMain, "main");
    R_DEF_CLOCK(prof, clkSim, "simulation");
    R_DEF_CLOCK(prof, clkPlan, "planner");
    R_DEF_CLOCK(prof, clkTelem, "telemetry");

    clkSim->reset();
    while (robot.step(TIME_STEP) != -1) {
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

        fra->setVelocity(axisSpeed);
        fla->setVelocity(axisSpeed);
        bra->setVelocity(axisSpeed);
        bla->setVelocity(axisSpeed);

        frw->setVelocity(wheelSpeed);
        flw->setVelocity(wheelSpeed);
        brw->setVelocity(wheelSpeed);
        blw->setVelocity(wheelSpeed);

        R_PROFILE_STEP(clkTelem, {
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
