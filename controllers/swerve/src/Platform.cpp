#include "Platform.hpp"

namespace swerve {
    using std::make_shared;

    SwerveDrive::SwerveDrive(Motor * axisMotor,
                             Motor * wheelMotor,
                             PositionSensor * axisEncoder,
                             PositionSensor * wheelEncoder)
        : axisMotor(axisMotor),
          wheelMotor(wheelMotor),
          axisEncoder(axisEncoder),
          wheelEncoder(wheelEncoder) {

        axisMotor->setPosition(INFINITY);
        wheelMotor->setPosition(INFINITY);

        axisMotor->setVelocity(0.0);
        wheelMotor->setVelocity(0.0);
    }

    void SwerveDrive::enable(int samplingPeriod) {
        axisEncoder->enable(samplingPeriod);
        wheelEncoder->enable(samplingPeriod);
    }

    void SwerveDrive::disable() {
        axisEncoder->disable();
        wheelEncoder->disable();
    }

    json SwerveDrive::getTelemetry() const {
        return json {
            {"axis",
             {
                 {"velocity", axisMotor->getVelocity()},
                 {"position", axisEncoder->getValue()},
             }},
            {"wheel",
             {
                 {"velocity", wheelMotor->getVelocity()},
                 {"position", wheelEncoder->getValue()},
             }},
        };
    }

    SwerveDrive::Ptr SwerveDrive::fromRobot(Robot & robot,
                                            const string & driveName) {
        auto * axisMotor = robot.getMotor(driveName + " axis motor");
        auto * wheelMotor = robot.getMotor(driveName + " wheel motor");
        auto * axisEncoder = robot.getPositionSensor(driveName + " axis sensor");
        auto * wheelEncoder = robot.getPositionSensor(driveName + " wheel sensor");
        return make_shared<SwerveDrive>(axisMotor, wheelMotor, axisEncoder,
                                        wheelEncoder);
    }
}

namespace swerve {
    Platform::Platform() : robot(), lastStep(0.0) {
        gps = robot.getGPS("gps");
        imu = robot.getInertialUnit("imu");

        frontRight = SwerveDrive::fromRobot(robot, "front right drive");
        frontLeft = SwerveDrive::fromRobot(robot, "front left drive");
        backRight = SwerveDrive::fromRobot(robot, "back right drive");
        backLeft = SwerveDrive::fromRobot(robot, "back left drive");
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
        frontRight->enable(samplingPeriod);
        frontLeft->enable(samplingPeriod);
        backRight->enable(samplingPeriod);
        backLeft->enable(samplingPeriod);
    }

    void Platform::disable() {
        gps->disable();
        imu->disable();
        frontRight->disable();
        frontLeft->disable();
        backRight->disable();
        backLeft->disable();
    }

    json Platform::getTelemetry() const {
        json motorData = {
            {"front_right", frontRight->getTelemetry()},
            {"front_left", frontLeft->getTelemetry()},
            {"back_right", backRight->getTelemetry()},
            {"back_left", backLeft->getTelemetry()},
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
