#pragma once

#include <memory>

#include "Motor.hpp"
#include "rclcpp/macros.hpp"
#include "util/PID.hpp"
#include "util/Telemetry.hpp"
#include "util/ramp.hpp"

namespace base {
    using util::TelemetrySender;
    using util::Ramp;
    using json = util::json;

    struct MotorProfile {
        double driveAccel = 1.0;
        double steerAccel = 1.0;
    };

    class SwerveDrive : public TelemetrySender {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(SwerveDrive)

    private:
        DriveMotor::SharedPtr drive;
        ServoMotor::SharedPtr servo;
        Ramp driveRamp;
        Ramp servoRamp;
        bool useProfile;
        double lastTime;
        double driveTarget;
        double steerTarget;

    public:
        SwerveDrive(const DriveMotor::SharedPtr & drive, const ServoMotor::SharedPtr & servo);

        SwerveDrive(const DriveMotor::SharedPtr & drive, const ServoMotor::SharedPtr & servo, const MotorProfile & profile);

        void enable(int samplingPeriod);

        void disable();

        void update(double time);

        void setSteer(double steer);

        double getSteerTarget() const;

        double getSteer() const;

        bool atSteerTarget(double tolerance = 0.01) const;

        double getMaxDriveVelocity() const;

        void setDriveVelocity(double velocity);

        void setDrivePower(double power);

        double getDriveVelocity() const;

        json getTelemetry() const override;
    };
}
