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
    using sim_time::now;
    using util::json;
    using util::Ramp;
    using util::TelemetrySender;

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

    public:
        TankController(const SwerveDrive::WeakPtr & leftDrive, const SwerveDrive::WeakPtr & rightDrive, double accel = 1.0)
            : Controller(leftDrive, rightDrive) {}

        void update(double time) override {
            if (auto left = leftDrive.lock()) {
                left->setSteer(0);
                left->setDrivePower(cmdPower - cmdSpin);
            }
            if (auto right = rightDrive.lock()) {
                right->setSteer(0);
                right->setDrivePower(cmdPower + cmdSpin);
            }

            Controller::update(time);
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
            double realDir = cmdDirection;
            while (realDir < -M_PI) {
                realDir += 2 * M_PI;
            }
            while (realDir > M_PI) {
                realDir -= 2 * M_PI;
            }

            double leftPower = cmdPower;
            double rightPower = cmdPower;

            double leftDirection = 0;
            double rightDirection = 0;

            if (realDir < -3 * M_PI_4) {
                leftDirection = lerp(std::abs(cmdSpin), realDir, -M_PI);
                rightDirection = lerp(std::abs(cmdSpin), realDir, -M_PI);

                if (cmdSpin >= 0) {
                    leftPower = lerp(cmdSpin, cmdPower, -cmdPower);
                }
                else {
                    rightPower = lerp(-cmdSpin, cmdPower, -cmdPower);
                }
            }
            else if (realDir < -M_PI_4) {
                leftDirection = lerp(std::abs(cmdSpin), realDir, 0);
                rightDirection = lerp(std::abs(cmdSpin), realDir, -M_PI);
            }
            else if (realDir < M_PI_4) {
                leftDirection = lerp(std::abs(cmdSpin), realDir, 0);
                rightDirection = lerp(std::abs(cmdSpin), realDir, 0);

                if (cmdSpin >= 0) {
                    leftPower = lerp(cmdSpin, cmdPower, -cmdPower);
                }
                else {
                    rightPower = lerp(-cmdSpin, cmdPower, -cmdPower);
                }
            }
            else if (realDir < 3 * M_PI_4) {
                leftDirection = lerp(std::abs(cmdSpin), realDir, M_PI);
                rightDirection = lerp(std::abs(cmdSpin), realDir, 0);
            }
            else {
                leftDirection = lerp(std::abs(cmdSpin), realDir, M_PI);
                rightDirection = lerp(std::abs(cmdSpin), realDir, M_PI);

                if (cmdSpin >= 0) {
                    leftPower = lerp(cmdSpin, cmdPower, -cmdPower);
                }
                else {
                    rightPower = lerp(-cmdSpin, cmdPower, -cmdPower);
                }
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
