#pragma once

#include <memory>
#include <string>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include "util/Telemetry.hpp"

namespace swerve {
    using namespace webots;
    using std::string;
    using std::shared_ptr;
    using util::TelemetrySender;
    using json = util::json;

    struct SwerveDrive : public TelemetrySender {
        using Ptr = shared_ptr<SwerveDrive>;
        using ConstPtr = const shared_ptr<SwerveDrive>;

        Motor * axisMotor;
        Motor * wheelMotor;

        PositionSensor * axisEncoder;
        PositionSensor * wheelEncoder;

        SwerveDrive(Motor * axisMotor,
                    Motor * wheelMotor,
                    PositionSensor * axisEncoder,
                    PositionSensor * wheelEncoder);

        void enable(int samplingPeriod);

        void disable();

        json getTelemetry() const override;

        static Ptr fromRobot(Robot & robot, const string & driveName);
    };

    class Platform : public TelemetrySender {
        int lastStep;

    public:
        using Ptr = shared_ptr<Platform>;
        using ConstPtr = const shared_ptr<Platform>;

    public:
        Robot robot;
        SwerveDrive::Ptr frontRight;
        SwerveDrive::Ptr frontLeft;
        SwerveDrive::Ptr backRight;
        SwerveDrive::Ptr backLeft;
        GPS * gps;
        InertialUnit * imu;

    public:
        Platform();

        int step(int duration);

        int getSamplingPeriod() const;

        double dt() const;

        void enable(int samplingPeriod);

        void disable();

        json getTelemetry() const override;
    };
}
