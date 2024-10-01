#include "base/Motor.hpp"

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
        motor->setVelocity(velocity);
    }

    void DriveMotor::setPower(double power) {
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

    json ServoMotor::getTelemetry() const {
        return json {
            {"target", getTarget()},
            {"position", getPosition()},
            {"velocity", getVelocity()},
        };
    }
}
