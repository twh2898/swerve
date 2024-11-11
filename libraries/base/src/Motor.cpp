#include "base/Motor.hpp"

#include <algorithm>
#include <cmath>

namespace base {
    DriveMotor::DriveMotor(webots::Motor * motor)
        : motor(motor) {
        motor->setPosition(INFINITY);
        motor->setVelocity(0.0);
    }

    double DriveMotor::getMaxVelocity() const {
        return motor->getMaxVelocity();
    }

    void DriveMotor::setVelocity(double velocity) {
        double max = getMaxVelocity();
        velocity = std::clamp(velocity, -max, max);
        motor->setVelocity(velocity);
    }

    void DriveMotor::setPower(double power) {
        power = std::clamp(power, -1.0, 1.0);
        setVelocity(power * getMaxVelocity());
    }

    double DriveMotor::getVelocity() const {
        return motor->getVelocity();
    }

    json DriveMotor::getTelemetry() const {
        return json {
            {"velocity", getVelocity()},
        };
    }
}

namespace base {
    ServoMotor::ServoMotor(webots::Motor * motor,
                           webots::PositionSensor * encoder)
        : motor(motor), encoder(encoder), target(0.0) {
        motor->setPosition(target);
        motor->setVelocity(motor->getMaxVelocity());
    }

    void ServoMotor::enable(int samplingPeriod) {
        encoder->enable(samplingPeriod);
    }

    void ServoMotor::disable() {
        encoder->disable();
    }

    void ServoMotor::setTarget(double newTarget) {
        target = newTarget;
        motor->setPosition(target);
    }

    double ServoMotor::getTarget() const {
        return target;
    }

    double ServoMotor::getPosition() const {
        return encoder->getValue();
    }

    double ServoMotor::getVelocity() const {
        return motor->getVelocity();
    }

    bool ServoMotor::atTarget(double tolerance) const {
        return std::abs(getTarget() - getPosition()) <= tolerance;
    }

    json ServoMotor::getTelemetry() const {
        return json {
            {"target", getTarget()},
            {"position", getPosition()},
            {"velocity", getVelocity()},
        };
    }
}
