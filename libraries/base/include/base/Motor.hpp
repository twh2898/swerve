#pragma once

#include <memory>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include "util/Telemetry.hpp"

namespace base {
    using std::shared_ptr;
    using util::TelemetrySender;
    using json = util::json;

    class DriveMotor : public TelemetrySender {
    public:
        using Ptr = shared_ptr<DriveMotor>;
        using ConstPtr = const shared_ptr<DriveMotor>;

    private:
        webots::Motor * motor;

    public:
        DriveMotor(webots::Motor * motor);

        double getMaxVelocity() const;

        /// on the scale [-maxVel, maxVel]
        void setVelocity(double velocity);

        /// on the scale [-1, 1]
        void setPower(double power);

        double getVelocity() const;

        json getTelemetry() const override;
    };

    class ServoMotor : public TelemetrySender {
    public:
        using Ptr = shared_ptr<ServoMotor>;
        using ConstPtr = const shared_ptr<ServoMotor>;

    private:
        webots::Motor * motor;
        webots::PositionSensor * encoder;
        double target;

    public:
        ServoMotor(webots::Motor * motor, webots::PositionSensor * encoder);

        void enable(int samplingPeriod);

        void disable();

        void setTarget(double newTarget);

        double getTarget() const;

        double getPosition() const;

        double getVelocity() const;

        json getTelemetry() const override;
    };
}
