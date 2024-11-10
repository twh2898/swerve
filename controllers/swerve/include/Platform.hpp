#pragma once

#include <memory>
#include <string>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include "rclcpp/macros.hpp"
#include "base/Motor.hpp"
#include "base/SwerveDrive.hpp"
#include "util/Telemetry.hpp"

namespace swerve {
    using std::string;
    using util::TelemetrySender;
    using base::SwerveDrive;
    using json = util::json;

    class Platform : public TelemetrySender {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(Platform)

    private:
        int lastStep;

    public:
        webots::Robot robot;
        SwerveDrive::SharedPtr rightDrive;
        SwerveDrive::SharedPtr leftDrive;
        webots::GPS * gps;
        webots::InertialUnit * imu;

        Platform();

        int step(int duration);

        int getSamplingPeriod() const;

        double dt() const;

        void enable(int samplingPeriod);

        void disable();

        void tank(double leftPower, double rightPower);

        void tank(double leftPower, double rightPower, double steer);

        void spin(double power);

        void bike(double power, double steer);

        json getTelemetry() const override;
    };
}
