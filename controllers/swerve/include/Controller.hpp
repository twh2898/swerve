#pragma once

#include <algorithm>

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

    public:
        Controller(const SwerveDrive::WeakPtr & leftDrive, const SwerveDrive::WeakPtr & rightDrive)
            : leftDrive(leftDrive), rightDrive(rightDrive) {}

        virtual ~Controller() {}

        virtual void update(double time) {}

        virtual void spin(double power) = 0;

        virtual void drive(double power, double direction) = 0;

        virtual void primeDirection(double direction) {}

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
        double spinRate;
        double power;
        Ramp lRamp;
        Ramp rRamp;

        void updateTarget() {
            lRamp.setTarget(power - spinRate, now());
            rRamp.setTarget(power + spinRate, now());
        }

    public:
        TankController(const SwerveDrive::WeakPtr & leftDrive, const SwerveDrive::WeakPtr & rightDrive, double accel = 1.0)
            : Controller(leftDrive, rightDrive),
              spinRate(0.0),
              power(0.0),
              lRamp(accel, now()),
              rRamp(accel, now()) {}

        void update(double time) override {
            if (auto left = leftDrive.lock()) {
                left->setDrivePower(lRamp.getValue(time));
            }

            if (auto right = rightDrive.lock()) {
                right->setDrivePower(rRamp.getValue(time));
            }
        }

        void setAccel(double accel) {
            lRamp.setSlope(accel, sim_time::now());
            rRamp.setSlope(accel, sim_time::now());
        }

        void spin(double rate) override {
            spinRate = rate;
            updateTarget();
        }

        // direction is ignored for tank controller
        void drive(double power, double direction) override {
            this->power = power;
            updateTarget();
        }

        json getTelemetry() const override {
            json data = Controller::getTelemetry();

            data["spin"] = spinRate;
            data["power"] = power;

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
}
