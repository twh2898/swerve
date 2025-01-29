#pragma once

#include <memory>
#include <string>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include "Controller.hpp"
#include "base/Motor.hpp"
#include "base/SwerveDrive.hpp"
#include "rclcpp/macros.hpp"
#include "util/Telemetry.hpp"

namespace swerve {
    using base::MotorProfile;
    using base::SwerveDrive;
    using std::string;
    using util::TelemetrySender;
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
        Controller::SharedPtr controller;

        Platform();

        Platform(const MotorProfile & profile);

        int step(int duration);

        int getSamplingPeriod() const;

        double getTime() const;

        double dt() const;

        void enable(int samplingPeriod);

        void disable();

        void update();

        void tank(double leftPower, double rightPower);

        void tank(double leftPower, double rightPower, double steer);

        void spin(double power);

        void bike(double power, double steer);

        json getTelemetry() const override;
    };
}
