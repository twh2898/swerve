#pragma once

#include "Platform.hpp"
#include "State.hpp"

namespace swerve {
    class SpinState : public State {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(SpinState)

        SpinState()
            : State("spin") {}

        void enter(const Platform::SharedPtr & plat, StateMachine * sm) override {
            plat->controller->spin(-0.125);
        }

        void step(const Platform::SharedPtr & plat, StateMachine * sm) override {
            const double * rpy = plat->imu->getRollPitchYaw();
            double z = rpy[2];

            if (z >= 1) {
                sm->transition("driveCurve");
                return;
            }
        }

        void exit(const Platform::SharedPtr & plat, StateMachine * sm) override {
            plat->controller->spin(0);
        }
    };

    class AlignWheelsState : public State {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(AlignWheelsState)

        AlignWheelsState()
            : State("align") {}

        void enter(const Platform::SharedPtr & plat, StateMachine * sm) override {
            plat->controller->drive(0.0, 0.8);
        }

        void step(const Platform::SharedPtr & plat, StateMachine * sm) override {
            if (plat->leftDrive->atSteerTarget() && plat->rightDrive->atSteerTarget()) {
                sm->transition("driveCurve");
                return;
            }
        }
    };

    class DriveCurveState : public State {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(DriveCurveState)

    private:
        double startTime;

    public:
        DriveCurveState()
            : State("driveCurve") {}

        void enter(const Platform::SharedPtr & plat, StateMachine * sm) override {
            plat->controller->drive(0.7, 0.8);
            plat->controller->spin(0.5);
        }

        void step(const Platform::SharedPtr & plat, StateMachine * sm) override {
            startTime = plat->robot.getTime();
            if (plat->robot.getTime() - startTime >= 3) {
                sm->transition("end");
                return;
            }
        }

        void exit(const Platform::SharedPtr & plat, StateMachine * sm) override {
            plat->controller->drive(0, 0);
        }
    };
}
