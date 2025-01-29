#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Platform.hpp"
#include "rclcpp/macros.hpp"

namespace swerve {
    using std::map;
    using std::string;
    using std::vector;

    class StateMachine;

    class State {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(State)

        const string name;

        State(const string & name);

        virtual ~State();

        virtual void enter(const Platform::SharedPtr & plat, StateMachine * sm);
        virtual void step(const Platform::SharedPtr & plat, StateMachine * sm);
        virtual void exit(const Platform::SharedPtr & plat, StateMachine * sm);
    };

    class StateMachine {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(StateMachine)

    private:
        Platform::SharedPtr plat;
        State::SharedPtr activeState;
        map<string, State::SharedPtr> states;

    public:
        StateMachine(const Platform::SharedPtr & plat,
                     const State::SharedPtr & active,
                     const vector<State::SharedPtr> & states);

        StateMachine(const Platform::SharedPtr & plat,
                     const State::SharedPtr & active,
                     const map<string, State::SharedPtr> & states);

        void step();

        void transition(const string & next);
    };
}
