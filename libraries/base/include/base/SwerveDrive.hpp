#pragma once

#include <memory>

#include "Motor.hpp"
#include "rclcpp/macros.hpp"
#include "util/PID.hpp"
#include "util/Telemetry.hpp"

namespace base {
    using util::TelemetrySender;
    using json = util::json;

    class SwerveDrive : public TelemetrySender {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(SwerveDrive)

    private:
        DriveMotor::SharedPtr drive;
        ServoMotor::SharedPtr servo;

    public:
        SwerveDrive(const DriveMotor::SharedPtr & drive, const ServoMotor::SharedPtr & servo);

        void enable(int samplingPeriod);

        void disable();

        void setSteer(double steer);

        double getSteerTarget() const;

        double getSteer() const;

        void setDriveVelocity(double velocity);

        void setDrivePower(double power);

        double getDriveVelocity() const;

        json getTelemetry() const override;
    };
}
