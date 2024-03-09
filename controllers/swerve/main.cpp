#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
using namespace webots;

#include <iostream>
#include <memory>
#include <vector>
using namespace std;

#define SPEED 5
#define TIME_STEP 64

int main() {
    int movementCounter = 10;
    int wheelSpeed, axisSpeed;

    Robot robot;

    auto fra = robot.getMotor("front right drive axis motor");
    auto fla = robot.getMotor("front left drive axis motor");
    auto bra = robot.getMotor("back right drive axis motor");
    auto bla = robot.getMotor("back left drive axis motor");

    auto frw = robot.getMotor("front right drive wheel motor");
    auto flw = robot.getMotor("front left drive wheel motor");
    auto brw = robot.getMotor("back right drive wheel motor");
    auto blw = robot.getMotor("back left drive wheel motor");

    fra->setPosition(INFINITY);
    fla->setPosition(INFINITY);
    bra->setPosition(INFINITY);
    bla->setPosition(INFINITY);
    frw->setPosition(INFINITY);
    flw->setPosition(INFINITY);
    brw->setPosition(INFINITY);
    blw->setPosition(INFINITY);

    fra->setVelocity(0.0);
    fla->setVelocity(0.0);
    bra->setVelocity(0.0);
    bla->setVelocity(0.0);

    frw->setVelocity(0.0);
    flw->setVelocity(0.0);
    brw->setVelocity(0.0);
    blw->setVelocity(0.0);

    while (robot.step(TIME_STEP) != -1) {
        if (movementCounter == 0) {
            // axisSpeed = 0;
            wheelSpeed = SPEED;
        }
        else {
            axisSpeed = SPEED / 5;
            wheelSpeed = 0;
            movementCounter--;
        }

        fra->setVelocity(axisSpeed);
        fla->setVelocity(axisSpeed);
        bra->setVelocity(axisSpeed);
        bla->setVelocity(axisSpeed);

        frw->setVelocity(wheelSpeed);
        flw->setVelocity(wheelSpeed);
        brw->setVelocity(wheelSpeed);
        blw->setVelocity(wheelSpeed);
    }

    return 0;
}
