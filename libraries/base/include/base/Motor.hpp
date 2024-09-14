#pragma once

#include <memory>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include "util/PID.hpp"
#include "util/Telemetry.hpp"

namespace base {
    using std::shared_ptr;
    using util::TelemetrySender;
    using util::PID;
    using json = util::json;

    class DriveMotor : public TelemetrySender {
    public:
        using Ptr = shared_ptr<DriveMotor>;
        using ConstPtr = const shared_ptr<DriveMotor>;

    private:
        webots::Motor * motor;

    public:
        DriveMotor(webots::Motor * motor);

        void setVelocity(double velocity);

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
        PID pid;
        double target;

    public:
        ServoMotor(webots::Motor * motor, webots::PositionSensor * encoder, PID & pid);

        void enable(int samplingPeriod);

        void disable();

        void setTarget(double newTarget);

        double getTarget() const;

        double getPosition() const;

        double getVelocity() const;

        void update(double dt);

        json getTelemetry() const override;
    };
}
