#ifndef COMMAND_H
#define COMMAND_H

#include <Servo.h>

#include "Utils.hpp"

class Command {
public:
    Command(ActuatorCommands& actuatorCmds) : actuatorCmds_(actuatorCmds) {}
    void setup();
    void commandGimbal(float angleX, float angleY);
    void adjustMotorThrustForBatteryVoltage(int16_t voltage_mV);
    void commandMotorsThrust(float thrustMotor, float rollTimingOffset);
    void extendLegs();
    void retractLegs();
    void setLedColor(bool red, bool green, bool blue);

private:
    int thrustToTiming(float thrust_gram);
    void doRollControlAndCommandMotor(float roll_rate_rps);
    Servo leg1;
    Servo leg2;
    Servo leg3;

    Servo gimbalX;
    Servo gimbalY;

    Servo motor1;
    Servo motor2;

    ActuatorCommands& actuatorCmds_;

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

    static constexpr float MAX_GIMBAL_ANGLE = 5.0f;  // degrees

    float currentGimbalAngleX = 0.0f, currentGimbalAngleY = 0.0f;
    float servoAngleX, servoAngleY;
    int timingMotor1, timingMotor2, timingGimbalX, timingGimbalY;
};

#endif