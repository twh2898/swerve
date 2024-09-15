#pragma once

#include <memory>

#include "Platform.hpp"

namespace swerve {
    using std::shared_ptr;

    enum class State {
        BEGIN,
        STATE1,
        STATE2,
        END,
    };

    class StateMachine {
    public:
        using Ptr = shared_ptr<StateMachine>;
        using ConstPtr = const shared_ptr<StateMachine>;

    protected:
        Platform::Ptr plat;
        State state;

        void begin() {
            state = State::STATE1;
        }

        void state1() {
            if (plat->robot.getTime() >= 1) {
                state = State::STATE2;
            }

            plat->leftDrive->setSteer(1);
            plat->rightDrive->setSteer(1);

            plat->leftDrive->setDriveVelocity(0);
            plat->rightDrive->setDriveVelocity(0);
        }

        void state2() {
            if (plat->robot.getTime() >= 3) {
                state = State::END;
            }

            plat->leftDrive->setSteer(1);
            plat->rightDrive->setSteer(1);

            plat->leftDrive->setDriveVelocity(5);
            plat->rightDrive->setDriveVelocity(5);
        }

        void end() {
            plat->leftDrive->setDriveVelocity(0);
            plat->rightDrive->setDriveVelocity(0);
        }

    public:
        StateMachine(const Platform::Ptr & plat)
            : state(State::BEGIN), plat(plat) {}

        void step() {
            switch (state) {
                case State::BEGIN:
                    begin();
                    break;
                case State::STATE1:
                    state1();
                    break;
                case State::STATE2:
                    state2();
                    break;
                case State::END:
                default:
                    end();
                    break;
            }
        }
    };
}
