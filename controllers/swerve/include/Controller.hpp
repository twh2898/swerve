#pragma once

#include <algorithm>
#include <cmath>

#include "base/SwerveDrive.hpp"
#include "rclcpp/macros.hpp"
#include "util/Telemetry.hpp"
#include "util/ramp.hpp"
#include "util/sim_time.hpp"

namespace swerve {
    using base::SwerveDrive;
    using util::TelemetrySender;
    using util::json;
    using util::Ramp;
    using sim_time::now;

    class Controller : public TelemetrySender {
    public:
        RCLCPP_SMART_PTR_ALIASES_ONLY(Controller)

    protected:
        SwerveDrive::WeakPtr leftDrive;
        SwerveDrive::WeakPtr rightDrive;

        double cmdSpin;
        double cmdPower;
        double cmdDirection;

    public:
        Controller(const SwerveDrive::WeakPtr & leftDrive, const SwerveDrive::WeakPtr & rightDrive)
            : leftDrive(leftDrive), rightDrive(rightDrive), cmdSpin(0.0), cmdPower(0.0), cmdDirection(0.0) {}

        virtual ~Controller() {}

        virtual void update(double time) {
            if (auto left = leftDrive.lock()) {
                left->update(time);
            }
            if (auto right = rightDrive.lock()) {
                right->update(time);
            }
        }

        double getSpin() const {
            return cmdSpin;
        }

        void setSpin(double spin) {
            cmdSpin = spin;
        }

        double getPower() const {
            return cmdPower;
        }

        void setPower(double power) {
            cmdPower = power;
        }

        double getDirection() const {
            return cmdDirection;
        }

        void setDirection(double direction) {
            cmdDirection = direction;
        }

        void drive(double power, double direction, double spin) {
            cmdPower = power;
            cmdDirection = direction;
            cmdSpin = spin;
        }

        json getTelemetry() const override {
            json leftData;
            if (auto left = leftDrive.lock()) {
                leftData = left->getTelemetry();
            }

            json rightData;
            if (auto right = rightDrive.lock()) {
                rightData = right->getTelemetry();
            }

            return json {
                {"left", leftData},
                {"right", rightData},
                {"spin", getSpin()},
                {"power", getPower()},
                {"direction", getDirection()},
            };
        }
    };

    class TankController : public Controller {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(TankController)

    private:
        Ramp lRamp;
        Ramp rRamp;

        void updateTarget() {
            lRamp.setTarget(cmdPower - cmdSpin, now());
            rRamp.setTarget(cmdPower + cmdSpin, now());
        }

    public:
        TankController(const SwerveDrive::WeakPtr & leftDrive, const SwerveDrive::WeakPtr & rightDrive, double accel = 1.0)
            : Controller(leftDrive, rightDrive),
              lRamp(accel, now()),
              rRamp(accel, now()) {}

        void update(double time) override {
            updateTarget();

            if (auto left = leftDrive.lock()) {
                left->setDrivePower(lRamp.getValue(time));
                left->update(time);
            }

            if (auto right = rightDrive.lock()) {
                right->setDrivePower(rRamp.getValue(time));
                right->update(time);
            }
        }

        void setAccel(double accel) {
            lRamp.setSlope(accel, sim_time::now());
            rRamp.setSlope(accel, sim_time::now());
        }

        json getTelemetry() const override {
            json data = Controller::getTelemetry();

            data["leftRamp"] = {
                {"start", lRamp.getStart()},
                {"target", lRamp.getTarget()},
                {"slope", lRamp.getSlope()},
                {"value", lRamp.getValue(now())},
                {"startTime", lRamp.getStartTime()},
            };

            data["rightRamp"] = {
                {"start", rRamp.getStart()},
                {"target", rRamp.getTarget()},
                {"slope", rRamp.getSlope()},
                {"value", rRamp.getValue(now())},
                {"startTime", rRamp.getStartTime()},
            };

            return data;
        }
    };

    class FullController : public Controller {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(FullController)

    private:
        void updateTarget() {
            double dirmod = std::sin(cmdDirection) * M_PI_4 * cmdSpin;

            if (auto left = leftDrive.lock()) {
                float leftPower = cmdPower + cmdSpin * dirmod;
                left->setDrivePower(leftPower);
                left->setSteer(cmdDirection + dirmod);
                // left->setSteer(cmdDirection);
            }
            if (auto right = rightDrive.lock()) {
                float rightPower = cmdPower - cmdSpin * dirmod;
                right->setDrivePower(rightPower);
                right->setSteer(cmdDirection - dirmod);
                // right->setSteer(cmdDirection);
            }
        }

    public:
        FullController(const SwerveDrive::WeakPtr & leftDrive, const SwerveDrive::WeakPtr & rightDrive)
            : Controller(leftDrive, rightDrive) {}

        void update(double time) override {
            updateTarget();

            Controller::update(time);
        }
    };
}
