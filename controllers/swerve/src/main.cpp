#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
using namespace webots;

#include <stdexcept>
#include <vector>

#include "Platform.hpp"
#include "Program.hpp"
#include "State.hpp"
#include "util/Config.hpp"
#include "util/Profiler.hpp"
#include "util/Telemetry.hpp"
#include "util/log.hpp"
#include "util/sim_time.hpp"

using namespace std;
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

    MotorProfile mProfile {
        driveAccel : config.controller.driveAccel,
        steerAccel : config.controller.steerAccel,
    };

    Platform::SharedPtr platform = Platform::make_shared(mProfile);
    int time_step = config.sim.step;
    if (time_step < platform->robot.getBasicTimeStep())
        time_step = platform->robot.getBasicTimeStep();
    platform->enable(time_step);

    platform->controller = FullController::make_shared(platform->leftDrive, platform->rightDrive);

    State::SharedPtr alignState = AlignWheelsState::make_shared();
    State::SharedPtr driveState = DriveCurveState::make_shared();

    vector<State::SharedPtr> states {alignState, driveState};
    StateMachine::SharedPtr sm = StateMachine::make_shared(platform, alignState, states);

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

        sim_time::setTime(platform->getTime());

        R_PROFILE_STEP(clkPlan, {
            sm->step();
            platform->update();
        });

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
