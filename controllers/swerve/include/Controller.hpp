#pragma once

#include <algorithm>

#include "base/SwerveDrive.hpp"
#include "rclcpp/macros.hpp"
#include "util/Telemetry.hpp"

namespace swerve {
    using base::SwerveDrive;
    using util::TelemetrySender;
    using util::json;

    class Controller : public TelemetrySender {
    public:
        RCLCPP_SMART_PTR_ALIASES_ONLY(Controller)

    protected:
        SwerveDrive::WeakPtr leftDrive;
        SwerveDrive::WeakPtr rightDrive;

    public:
        Controller(const SwerveDrive::WeakPtr & leftDrive, const SwerveDrive::WeakPtr & rightDrive)
            : leftDrive(leftDrive), rightDrive(rightDrive) {}

        virtual ~Controller() {}

        virtual void spin(double power) = 0;

        virtual void drive(double power, double direction) = 0;

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
            };
        }
    };

    class TankController : public Controller {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(TankController)

    private:
        double lastDrivePower;
        double lastSpin;

        void driveAndSpin(double power, double spinPower) {
            double leftPower = std::clamp(power - spinPower, -1.0, 1.0);
            double rightPower = std::clamp(power + spinPower, -1.0, 1.0);

            if (auto left = leftDrive.lock()) {
                left->setDrivePower(leftPower);
                left->setSteer(0);
            }

            if (auto right = rightDrive.lock()) {
                right->setDrivePower(rightPower);
                right->setSteer(0);
            }
        }

    public:
        using Controller::Controller;

        void spin(double power) override {
            driveAndSpin(lastDrivePower, power);
            lastSpin = power;
        }

        void drive(double power, double direction) override {
            driveAndSpin(power, lastSpin);
            lastDrivePower = power;
        }
    };
}
