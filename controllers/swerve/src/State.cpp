#include "State.hpp"

#include <util/log.hpp>

namespace swerve {
    State::State(const string & name)
        : name(name) {}

    State::~State() {}

    void State::enter(const Platform::SharedPtr & plat, StateMachine * sm) {}

    void State::step(const Platform::SharedPtr & plat, StateMachine * sm) {}

    void State::exit(const Platform::SharedPtr & plat, StateMachine * sm) {}
}

namespace swerve {
    class InitState : public State {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(InitState)

    private:
        State::SharedPtr firstState;

    public:
        InitState(const State::SharedPtr & firstState)
            : State("_init"), firstState(firstState) {}

        void step(const Platform::SharedPtr & plat, StateMachine * sm) override {
            sm->transition(firstState->name);
        }
    };
}

namespace swerve {
    StateMachine::StateMachine(const Platform::SharedPtr & plat,
                               const State::SharedPtr & active,
                               const vector<State::SharedPtr> & states)
        : plat(plat), activeState(InitState::make_shared(active)) {

        for (auto & state : states) {
            this->states[state->name] = state;
        }

        if (this->states.find("end") == this->states.end()) {
            this->states["end"] = State::make_shared("end");
        }
    }

    StateMachine::StateMachine(const Platform::SharedPtr & plat,
                               const State::SharedPtr & active,
                               const map<string, State::SharedPtr> & states)
        : plat(plat), activeState(active), states(states) {}

    void StateMachine::step() {
        activeState->step(plat, this);
    }

    void StateMachine::transition(const string & next) {
        util::Logging::Planning->info("Changing states from {} to {}", activeState->name, next);
        activeState->exit(plat, this);
        auto & state = states[next];

        activeState = state;
        state->enter(plat, this);
    }
}
