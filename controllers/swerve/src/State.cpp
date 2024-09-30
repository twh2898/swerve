#include "State.hpp"

#include <util/log.hpp>

namespace swerve {
    State::State(const string & name)
        : name(name) {}

    State::~State() {}

    void State::enter(const Platform::Ptr & plat, StateMachine * sm) {}

    void State::step(const Platform::Ptr & plat, StateMachine * sm) {}

    void State::exit(const Platform::Ptr & plat, StateMachine * sm) {}
}

namespace swerve {
    class InitState : public State {
    public:
        using Ptr = shared_ptr<InitState>;

    private:
        State::Ptr firstState;

    public:
        InitState(const State::Ptr & firstState)
            : State("_init"), firstState(firstState) {}

        void step(const Platform::Ptr & plat, StateMachine * sm) override {
            sm->transition(firstState->name);
        }
    };
}

namespace swerve {
    StateMachine::StateMachine(const Platform::Ptr & plat,
                               const State::Ptr & active,
                               const vector<State::Ptr> & states)
        : plat(plat), activeState(make_shared<InitState>(active)) {

        for (auto & state : states) {
            this->states[state->name] = state;
        }

        if (this->states.find("end") == this->states.end()) {
            this->states["end"] = make_shared<State>("end");
        }
    }

    StateMachine::StateMachine(const Platform::Ptr & plat,
                               const State::Ptr & active,
                               const map<string, State::Ptr> & states)
        : plat(plat), activeState(active), states(states) {}

    void StateMachine::step() {
        activeState->step(plat, this);
    }

    void StateMachine::transition(const string & next) {
        util::Logging::Planning->info("Changing states from {} to {}", activeState->name, next);
        activeState->exit(plat, this);
        auto & state = states[next];

        state->enter(plat, this);
        activeState = state;
    }
}
