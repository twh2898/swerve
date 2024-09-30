#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Platform.hpp"

namespace swerve {
    using std::shared_ptr;
    using std::make_shared;
    using std::string;
    using std::map;
    using std::vector;

    class StateMachine;

    class State {
    public:
        using Ptr = shared_ptr<State>;

        const string name;

        State(const string & name);

        virtual ~State();

        virtual void enter(const Platform::Ptr & plat, StateMachine * sm);
        virtual void step(const Platform::Ptr & plat, StateMachine * sm);
        virtual void exit(const Platform::Ptr & plat, StateMachine * sm);
    };

    class StateMachine {
    public:
        using Ptr = shared_ptr<StateMachine>;

    private:
        Platform::Ptr plat;
        State::Ptr activeState;
        map<string, State::Ptr> states;

    public:
        StateMachine(const Platform::Ptr & plat,
                     const State::Ptr & active,
                     const vector<State::Ptr> & states);

        StateMachine(const Platform::Ptr & plat,
                     const State::Ptr & active,
                     const map<string, State::Ptr> & states);

        void step();

        void transition(const string & next);
    };
}
