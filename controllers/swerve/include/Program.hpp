#pragma once

#include "Platform.hpp"
#include "State.hpp"
#include "util/Config.hpp"
#include "util/log.hpp"

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
            plat->controller->drive(0.0, 3.14 / 2);
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
            startTime = plat->getTime();

            // plat->controller->drive(0.7, 3.14/2);
            plat->controller->drive(0.0, 0);
            plat->controller->spin(0.5);
            // plat->controller->spin(0.0);
        }

        void step(const Platform::SharedPtr & plat, StateMachine * sm) override {
            if (plat->getTime() - startTime >= 3) {
                sm->transition("end");
                return;
            }
        }

        void exit(const Platform::SharedPtr & plat, StateMachine * sm) override {
            plat->controller->drive(0, 0);
        }
    };

    class MissionState : public State {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(MissionState)

    private:
        double startTime;
        double stepEnd;
        vector<util::Mission> mission;
        int stepIndex;

    public:
        MissionState(const vector<util::Mission> & mission)
            : State("mission"), mission(mission), stepIndex(0) {}

        void enter(const Platform::SharedPtr & plat, StateMachine * sm) override {
            startTime = plat->getTime();
            if (mission.empty()) {
                sm->transition("end");
                return;
            }
            stepEnd = startTime + mission.at(0).duration;
            util::Logging::MC->info("Mission activating stage {} / {}", stepIndex, mission.size());
        }

        void step(const Platform::SharedPtr & plat, StateMachine * sm) override {
            if (stepIndex >= mission.size()) {
                sm->transition("end");
                return;
            }

            while (plat->getTime() >= stepEnd) {
                stepIndex++;
                util::Logging::MC->info("Mission activating stage {} / {}", stepIndex, mission.size());

                if (stepIndex >= mission.size()) {
                    sm->transition("end");
                    return;
                }

                stepEnd += mission.at(stepIndex).duration;
            }

            auto & currStep = mission.at(stepIndex);

            plat->controller->drive(currStep.power, currStep.direction);
            plat->controller->spin(currStep.spin);
        }

        void exit(const Platform::SharedPtr & plat, StateMachine * sm) override {
            plat->controller->drive(0, 0);
        }
    };
}
