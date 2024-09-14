#include "base/SwerveDrive.hpp"

namespace base {
    SwerveDrive::SwerveDrive(const DriveMotor::Ptr & drive, const ServoMotor::Ptr & servo)
        : drive(drive), servo(servo) {
    }

    void SwerveDrive::enable(int samplingPeriod) {
        servo->enable(samplingPeriod);
    }

    void SwerveDrive::disable() {
        servo->disable();
    }

    void SwerveDrive::setTarget(double newTarget) {
        servo->setTarget(newTarget);
    }

    double SwerveDrive::getTarget() const {
        return servo->getTarget();
    }

    double SwerveDrive::getAngle() const {
        return servo->getPosition();
    }

    void SwerveDrive::setVelocity(double velocity) {
        drive->setVelocity(velocity);
    }

    double SwerveDrive::getVelocity() const {
        return drive->getVelocity();
    }

    void SwerveDrive::update(double dt) {
        servo->update(dt);
    }

    json SwerveDrive::getTelemetry() const {
        return json {
            {"drive", drive->getTelemetry()},
            {"servo", servo->getTelemetry()},
        };
    }
}
