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
            cmdSpin = std::clamp(spin, -1.0, 1.0);
        }

        double getPower() const {
            return cmdPower;
        }

        void setPower(double power) {
            cmdPower = std::clamp(power, -1.0, 1.0);
        }

        double getDirection() const {
            return cmdDirection;
        }

        void setDirection(double direction) {
            cmdDirection = direction;
        }

        void drive(double power, double direction, double spin) {
            setPower(power);
            setDirection(direction);
            setSpin(spin);
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
        double lerp(double t, double a, double b) {
            return a + t * (b - a);
        }

        void driveTank() {
            double leftPower = cmdPower;
            double rightPower = cmdPower;

            if (cmdSpin >= 0) {
                leftPower = lerp(cmdSpin, cmdPower, -cmdPower);
            }
            else {
                rightPower = lerp(-cmdSpin, cmdPower, -cmdPower);
            }

            if (auto left = leftDrive.lock()) {
                left->setDrivePower(leftPower);
                double leftDirection = lerp(cmdSpin, cmdDirection, 0);
                left->setSteer(leftDirection);
            }

            if (auto right = rightDrive.lock()) {
                right->setDrivePower(rightPower);
                double rightDirection = lerp(cmdSpin, cmdDirection, 0);
                right->setSteer(rightDirection);
            }
        }

        void driveBike() {
            double leftPower = cmdPower;
            double rightPower = cmdPower;

            double leftDirection = lerp(cmdSpin, cmdDirection, 0);
            double rightDirection = lerp(cmdSpin, cmdDirection, M_PI);

            if (auto left = leftDrive.lock()) {
                left->setDrivePower(leftPower);
                left->setSteer(leftDirection);
            }

            if (auto right = rightDrive.lock()) {
                right->setDrivePower(rightPower);
                right->setSteer(rightDirection);
            }
        }

        void driveBoth() {
            double tank_mod = std::sin(cmdDirection);

            // double leftPower = lerp(tank_mod, cmdPower - cmdSpin, cmdPower);
            // double rightPower = lerp(tank_mod, cmdPower + cmdSpin, cmdPower);

            double leftPower = cmdPower;
            double rightPower = cmdPower;

            if (cmdSpin >= 0) {
                leftPower = lerp(cmdSpin, cmdPower, -cmdPower);
            }
            else {
                rightPower = lerp(-cmdSpin, cmdPower, -cmdPower);
            }

            double leftDirection;
            double rightDirection;

            if (cmdSpin >= 0) {
                leftDirection = lerp(cmdSpin, cmdDirection, 0);
                rightDirection = lerp(cmdSpin, cmdDirection, M_PI * 2);
            }
            else {
                leftDirection = lerp(-cmdSpin, cmdDirection, M_PI * 2);
                rightDirection = lerp(-cmdSpin, cmdDirection, 0);
            }

            if (auto left = leftDrive.lock()) {
                left->setDrivePower(leftPower);
                left->setSteer(leftDirection);
            }

            if (auto right = rightDrive.lock()) {
                right->setDrivePower(rightPower);
                right->setSteer(rightDirection);
            }
        }

        void updateTarget() {
            // driveTank();
            // driveBike();
            driveBoth();
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
