#include "base/SwerveDrive.hpp"

namespace base {
    SwerveDrive::SwerveDrive(const DriveMotor::SharedPtr & drive, const ServoMotor::SharedPtr & servo)
        : drive(drive), servo(servo) {
    }

    void SwerveDrive::enable(int samplingPeriod) {
        servo->enable(samplingPeriod);
    }

    void SwerveDrive::disable() {
        servo->disable();
    }

    void SwerveDrive::setSteer(double steer) {
        servo->setTarget(steer);
    }

    double SwerveDrive::getSteerTarget() const {
        return servo->getTarget();
    }

    double SwerveDrive::getSteer() const {
        return servo->getPosition();
    }

    double SwerveDrive::getMaxDriveVelocity() const {
        return drive->getMaxVelocity();
    }

    void SwerveDrive::setDriveVelocity(double velocity) {
        drive->setVelocity(velocity);
    }

    void SwerveDrive::setDrivePower(double power) {
        drive->setPower(power);
    }

    double SwerveDrive::getDriveVelocity() const {
        return drive->getVelocity();
    }

    json SwerveDrive::getTelemetry() const {
        return json {
            {"drive", drive->getTelemetry()},
            {"steer", servo->getTelemetry()},
        };
    }
}
