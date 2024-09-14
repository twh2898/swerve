#pragma once

#include <memory>
#include <string>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include "base/Motor.hpp"
#include "base/SwerveDrive.hpp"
#include "util/PID.hpp"
#include "util/Telemetry.hpp"

namespace swerve {
    using std::string;
    using std::shared_ptr;
    using util::TelemetrySender;
    using util::PID;
    using base::SwerveDrive;
    using json = util::json;

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
