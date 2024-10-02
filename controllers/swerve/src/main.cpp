#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
using namespace webots;

#include <stdexcept>
#include <vector>

#include "Platform.hpp"
#include "State.hpp"
#include "util/Config.hpp"
#include "util/Profiler.hpp"
#include "util/Telemetry.hpp"
#include "util/log.hpp"

using namespace std;
using namespace util;
using namespace swerve;

class State1 : public State {
public:
    State1()
        : State("state1") {}

    void step(const Platform::Ptr & plat, StateMachine * sm) override {
        const double * rpy = plat->imu->getRollPitchYaw();
        double z = rpy[2];

        if (z < 1) {
            plat->spin(-0.125);
        }
        else {
            plat->spin(0);
            sm->transition("state2");
        }
    }
};

class State2 : public State {
    double startTime;

public:
    State2()
        : State("state2") {}

    void enter(const Platform::Ptr & plat, StateMachine * sm) override {
        plat->tank(0.25, 0.25);
        startTime = plat->robot.getTime();
    }

    void step(const Platform::Ptr & plat, StateMachine * sm) override {
        if (plat->robot.getTime() - startTime >= 3) {
            sm->transition("end");
            return;
        }
    }

    void exit(const Platform::Ptr & plat, StateMachine * sm) override {
        plat->tank(0, 0);
    }
};

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

    State::Ptr state1 = make_shared<State1>();
    State::Ptr state2 = make_shared<State2>();

    vector<State::Ptr> states {state1, state2};
    StateMachine::Ptr sm = make_shared<StateMachine>(platform, state1, states);

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

        R_PROFILE_STEP(clkPlan, {
            sm->step();
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
