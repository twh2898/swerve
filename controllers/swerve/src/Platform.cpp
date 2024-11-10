#include "Platform.hpp"

#include <cmath>

namespace swerve {
    using base::DriveMotor;
    using base::ServoMotor;

    static SwerveDrive::SharedPtr driveFromRobot(webots::Robot & robot,
                                                 const string & driveName) {
        auto * axisMotor = robot.getMotor(driveName + " axis motor");
        auto * wheelMotor = robot.getMotor(driveName + " wheel motor");
        auto * axisEncoder = robot.getPositionSensor(driveName + " axis sensor");
        DriveMotor::SharedPtr drive = DriveMotor::make_shared(wheelMotor);
        ServoMotor::SharedPtr servo = ServoMotor::make_shared(axisMotor, axisEncoder);
        return SwerveDrive::make_shared(drive, servo);
    }

    Platform::Platform()
        : robot(), lastStep(0.0) {
        gps = robot.getGPS("gps");
        imu = robot.getInertialUnit("imu");

        rightDrive = driveFromRobot(robot, "right drive");
        leftDrive = driveFromRobot(robot, "left drive");
    }

    int Platform::step(int duration) {
        lastStep = duration;
        return robot.step(duration);
    }

    int Platform::getSamplingPeriod() const {
        return lastStep;
    }

    double Platform::dt() const {
        return (double)lastStep / 1000.0;
    }

    void Platform::enable(int samplingPeriod) {
        gps->enable(samplingPeriod);
        imu->enable(samplingPeriod);
        rightDrive->enable(samplingPeriod);
        leftDrive->enable(samplingPeriod);
    }

    void Platform::disable() {
        gps->disable();
        imu->disable();
        rightDrive->disable();
        leftDrive->disable();
    }

    void Platform::tank(double leftPower, double rightPower) {
        leftDrive->setDrivePower(leftPower);
        rightDrive->setDrivePower(rightPower);
    }

    void Platform::tank(double leftPower, double rightPower, double steer) {
        leftDrive->setDrivePower(leftPower);
        leftDrive->setSteer(steer);

        rightDrive->setDrivePower(rightPower);
        rightDrive->setSteer(steer);
    }

    void Platform::spin(double power) {
        tank(power, -power, 0);
    }

    void Platform::bike(double power, double steer) {
        leftDrive->setDrivePower(power);
        leftDrive->setSteer((M_PI / 2) - steer);

        rightDrive->setDrivePower(power);
        rightDrive->setSteer((M_PI / 2) + steer);
    }

    json Platform::getTelemetry() const {
        json motorData = {
            {"left", rightDrive->getTelemetry()},
            {"right", leftDrive->getTelemetry()},
        };

        auto * gpsV = gps->getValues();
        json gpsData = {
            {"x", gpsV[0]},
            {"y", gpsV[1]},
            {"z", gpsV[2]},
        };

        auto * imuV = imu->getRollPitchYaw();
        json imuData = {
            {"x", imuV[0]},
            {"y", imuV[1]},
            {"z", imuV[2]},
        };

        json sensorData = {
            {"gps", gpsData},
            {"imu", imuData},
        };

        return json {
            {"motor", motorData},
            {"sensors", sensorData},
            {"time", robot.getTime()},
            {"dt", dt()},
        };
    }
}
