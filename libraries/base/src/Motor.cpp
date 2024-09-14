#include "base/Motor.hpp"

namespace base {
    DriveMotor::DriveMotor(webots::Motor * motor)
        : motor(motor) {
        motor->setPosition(INFINITY);
        motor->setVelocity(0.0);
    }

    void DriveMotor::setVelocity(double velocity) {
        double max = motor->getMaxVelocity();
        if (velocity > max)
            velocity = max;
        if (velocity < -max)
            velocity = -max;

        motor->setVelocity(velocity);
    }

    double DriveMotor::getVelocity() const {
        return motor->getVelocity();
    }

    json DriveMotor::getTelemetry() const {
        return json {
            {"velocity", motor->getVelocity()},
        };
    }
}

namespace base {
    ServoMotor::ServoMotor(webots::Motor * motor,
                           webots::PositionSensor * encoder,
                           PID & pid)
        : motor(motor), encoder(encoder), pid(pid), target(0.0) {
        motor->setPosition(INFINITY);
        motor->setVelocity(0.0);
    }

    void ServoMotor::enable(int samplingPeriod) {
        encoder->enable(samplingPeriod);
    }

    void ServoMotor::disable() {
        encoder->disable();
    }

    void ServoMotor::setTarget(double newTarget) {
        target = newTarget;
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

    void ServoMotor::update(double dt) {
        double command = pid.calculate(dt, target, encoder->getValue());

        double max = motor->getMaxVelocity();
        if (command > max)
            command = max;
        if (command < -max)
            command = -max;

        motor->setVelocity(command);
    }

    json ServoMotor::getTelemetry() const {
        return json {
            {"target", target},
            {"position", getPosition()},
            {"velocity", getVelocity()},
            {"pid", pid.getTelemetry()},
        };
    }
}
