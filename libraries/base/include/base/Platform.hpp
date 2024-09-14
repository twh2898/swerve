#pragma once

#include <memory>
#include <string>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include "Motor.hpp"
#include "util/PID.hpp"
#include "util/Telemetry.hpp"

namespace swerve {
    using std::string;
    using std::shared_ptr;
    using util::TelemetrySender;
    using util::PID;
    using json = util::json;

    struct SwerveDrive : public TelemetrySender {
        using Ptr = shared_ptr<SwerveDrive>;
        using ConstPtr = const shared_ptr<SwerveDrive>;

        ServoMotor::Ptr axis;
        DriveMotor::Ptr wheel;

        SwerveDrive(webots::Motor * axisMotor,
                    webots::Motor * wheelMotor,
                    webots::PositionSensor * axisEncoder,
                    PID & pid);

        SwerveDrive(const ServoMotor::Ptr & axis, const DriveMotor::Ptr & wheel);

        void enable(int samplingPeriod);

        void disable();

        void update(double dt);

        json getTelemetry() const override;
    };

    class Platform : public TelemetrySender {
        int lastStep;

    public:
        using Ptr = shared_ptr<Platform>;
        using ConstPtr = const shared_ptr<Platform>;

    public:
        webots::Robot robot;
        SwerveDrive::Ptr frontRight;
        SwerveDrive::Ptr frontLeft;
        SwerveDrive::Ptr backRight;
        SwerveDrive::Ptr backLeft;
        webots::GPS * gps;
        webots::InertialUnit * imu;

    public:
        Platform(PID & swervePID);

        int step(int duration);

        int getSamplingPeriod() const;

        double dt() const;

        void enable(int samplingPeriod);

        void disable();

        json getTelemetry() const override;
    };
}
