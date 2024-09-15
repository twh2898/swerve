#include "Platform.hpp"

namespace swerve {
    using base::DriveMotor;
    using base::ServoMotor;
    using std::make_shared;

    static SwerveDrive::Ptr driveFromRobot(webots::Robot & robot,
                                           const string & driveName) {
        auto * axisMotor = robot.getMotor(driveName + " axis motor");
        auto * wheelMotor = robot.getMotor(driveName + " wheel motor");
        auto * axisEncoder = robot.getPositionSensor(driveName + " axis sensor");
        DriveMotor::Ptr drive = make_shared<DriveMotor>(wheelMotor);
        ServoMotor::Ptr servo = make_shared<ServoMotor>(axisMotor, axisEncoder);
        return make_shared<SwerveDrive>(drive, servo);
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
        int res = robot.step(duration);
        double dt = duration / 1000.0;
        // if (res != -1) {
        //     frontRight->update(dt);
        //     frontLeft->update(dt);
        //     backRight->update(dt);
        //     backLeft->update(dt);
        // }
        return res;
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
