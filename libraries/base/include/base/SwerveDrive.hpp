#pragma once

#include <memory>

#include "Motor.hpp"
#include "util/PID.hpp"
#include "util/Telemetry.hpp"

namespace base {
    using std::shared_ptr;
    using util::TelemetrySender;
    using json = util::json;

    class SwerveDrive : public TelemetrySender {
    public:
        using Ptr = shared_ptr<SwerveDrive>;
        using ConstPtr = const shared_ptr<SwerveDrive>;

    private:
        DriveMotor::Ptr drive;
        ServoMotor::Ptr servo;

    public:
        SwerveDrive(const DriveMotor::Ptr & drive, const ServoMotor::Ptr & servo);

        void enable(int samplingPeriod);

        void disable();

        void setTarget(double newTarget);

        double getTarget() const;

        double getAngle() const;

        void setVelocity(double velocity);

        double getVelocity() const;

        void update(double dt);

        json getTelemetry() const override;
    };
}
