#include "base/SwerveDrive.hpp"

#include "util/sim_time.hpp"

namespace base {
    using sim_time::now;

    SwerveDrive::SwerveDrive(const DriveMotor::SharedPtr & drive, const ServoMotor::SharedPtr & servo)
        : drive(drive), servo(servo), useProfile(false), lastTime(0.0) {}

    SwerveDrive::SwerveDrive(const DriveMotor::SharedPtr & drive, const ServoMotor::SharedPtr & servo, const MotorProfile & profile)
        : drive(drive), servo(servo), driveRamp(profile.driveAccel, now()), servoRamp(profile.steerAccel, now()), useProfile(true), lastTime(0.0) {}

    void SwerveDrive::enable(int samplingPeriod) {
        servo->enable(samplingPeriod);
    }

    void SwerveDrive::disable() {
        servo->disable();
    }

    void SwerveDrive::update(double time) {
        if (lastTime == 0.0) {
            lastTime = time;
        }

        if (useProfile) {
            drive->setPower(driveRamp.getValue(time));
            servo->setTarget(servoRamp.getValue(time));
        }
        else {
            drive->setPower(driveTarget);
            servo->setTarget(steerTarget);
        }

        lastTime = time;
    }

    void SwerveDrive::setSteer(double steer) {
        steerTarget = steer;
        if (useProfile) {
            servoRamp.setTarget(steerTarget, lastTime);
        }
    }

    double SwerveDrive::getSteerTarget() const {
        return steerTarget;
    }

    double SwerveDrive::getSteer() const {
        return servo->getPosition();
    }

    double SwerveDrive::getMaxDriveVelocity() const {
        return drive->getMaxVelocity();
    }

    void SwerveDrive::setDriveVelocity(double velocity) {
        setDrivePower(velocity / getMaxDriveVelocity());
    }

    void SwerveDrive::setDrivePower(double power) {
        driveTarget = power;
        if (useProfile) {
            driveRamp.setTarget(driveTarget, lastTime);
        }
    }

    double SwerveDrive::getDriveVelocity() const {
        return drive->getVelocity();
    }

    json SwerveDrive::getTelemetry() const {
        json data {
            {"drive", drive->getTelemetry()},
            {"steer", servo->getTelemetry()},
            {"useProfile", useProfile},
            {"lastTime", lastTime},
            {"driveTarget", driveTarget},
            {"steerTarget", steerTarget},
        };

        if (useProfile) {
            data["driveRamp"] = {
                {"start", driveRamp.getStart()},
                {"target", driveRamp.getTarget()},
                {"slope", driveRamp.getSlope()},
                {"value", driveRamp.getValue(now())},
                {"startTime", driveRamp.getStartTime()},
            };

            data["servoRamp"] = {
                {"start", servoRamp.getStart()},
                {"target", servoRamp.getTarget()},
                {"slope", servoRamp.getSlope()},
                {"value", servoRamp.getValue(now())},
                {"startTime", servoRamp.getStartTime()},
            };
        }

        return data;
    }
}
