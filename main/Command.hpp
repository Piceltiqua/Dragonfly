#ifndef COMMAND_H
#define COMMAND_H

#include <Servo.h>

#include "Utils.hpp"

class Command {
public:
    Command() {}
    void setup();
    void commandGimbal(float angleX, float angleY);
    void commandMotors(int throttleMotor1, int throttleMotor2);
    void extendLegs();
    void retractLegs();
    void setLedColor(bool red, bool green, bool blue);

private:
    Servo leg1;
    Servo leg2;
    Servo leg3;

    Servo gimbalX;
    Servo gimbalY;

    Servo motor1;
    Servo motor2;

    static constexpr int RED_PIN = 35;
    static constexpr int GREEN_PIN = 34;
    static constexpr int BLUE_PIN = 33;

    static constexpr int PIN_LEG_1 = 6;
    static constexpr int PIN_LEG_2 = 7;
    static constexpr int PIN_LEG_3 = 8;
    static constexpr int PIN_GIMBAL_X = 4;
    static constexpr int PIN_GIMBAL_Y = 5;
    static constexpr int PIN_MOTOR_1 = 2;
    static constexpr int PIN_MOTOR_2 = 3;

    static constexpr float MAX_GIMBAL_ANGLE = 6.0f;  // degrees

    float servoAngleGimbalX, servoAngleGimbalY;
    int timingMotor1, timingMotor2, timingGimbalX, timingGimbalY;
};

#endif