#include "Command.hpp"

void Command::setup() {
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    leg1.attach(6, 500, 2500);
    leg2.attach(7, 500, 2500);
    leg3.attach(8, 500, 2500);
    gimbalX.attach(4, 900, 2100);
    gimbalY.attach(5, 900, 2100);
    motor1.attach(2, 1100, 1940);
    motor2.attach(3, 1100, 1940);

    leg1.writeMicroseconds(730);
    leg2.writeMicroseconds(730);
    leg3.writeMicroseconds(800);
    motor1.writeMicroseconds(1100);
    motor2.writeMicroseconds(1100);
}

void Command::commandGimbal(float gimbalAngleX, float gimbalAngleY) {
    /*
        Commands the TVC servos at 250Hz such that the gimbal reaches the specified angles.
        THE ANGLES GIVEN ARE NOT THE SERVO ANGLES.
        gimbalAngleX: Desired angle for the X-axis (in degrees).
        gimbalAngleY: Desired angle for the Y-axis (in degrees).
        */

    if (gimbalAngleX > MAX_GIMBAL_ANGLE) {
        gimbalAngleX = MAX_GIMBAL_ANGLE;
    }
    if (gimbalAngleX < -MAX_GIMBAL_ANGLE) {
        gimbalAngleX = -MAX_GIMBAL_ANGLE;
    }
    if (gimbalAngleY > MAX_GIMBAL_ANGLE) {
        gimbalAngleY = MAX_GIMBAL_ANGLE;
    }
    if (gimbalAngleY < -MAX_GIMBAL_ANGLE) {
        gimbalAngleY = -MAX_GIMBAL_ANGLE;
    }
    servoAngleGimbalX = -0.00529 * pow(gimbalAngleX, 3) + 0.0233 * pow(gimbalAngleX, 2) - 5.60 * gimbalAngleX + 1.37;
    servoAngleGimbalY = 0.0117 * pow(gimbalAngleY, 3) - 0.0274 * pow(gimbalAngleY, 2) + 5.28 * gimbalAngleY + 0.108;
    Serial.print("Gimbal servo angles: ");
    Serial.print(servoAngleGimbalX);
    Serial.print(", ");
    Serial.println(servoAngleGimbalY);
    gimbalX.writeMicroseconds(map(servoAngleGimbalX, -60.0f, 60.0f, 900, 2100));
    gimbalY.writeMicroseconds(map(servoAngleGimbalY, -60.0f, 60.0f, 900, 2100));
    //gimbalX.writeMicroseconds(1400);
    //gimbalY.writeMicroseconds(900);
}

void Command::commandMotors(int throttleMotor1, int throttleMotor2) {
    /*
        Commands the ESCs for the two BLDC motors.
        throttleMotor1: Throttle for motor 1 (top motor) (0-100%).
        throttleMotor2: Throttle for motor 2 (bottom motor) (0-100%).
        */
    if (throttleMotor1 > 100) {
        throttleMotor1 = 100;
    }
    if (throttleMotor1 < 0) {
        throttleMotor1 = 0;
    }
    if (throttleMotor2 > 100) {
        throttleMotor2 = 100;
    }
    if (throttleMotor2 < 0) {
        throttleMotor2 = 0;
    }

    timingMotor1 = map(throttleMotor1, 0, 100, 1100, 1940);
    timingMotor2 = map(throttleMotor2, 0, 100, 1100, 1940);
    motor1.writeMicroseconds(timingMotor1);
    motor2.writeMicroseconds(timingMotor2);
}

void Command::extendLegs() {
    leg1.writeMicroseconds(700);
    leg2.writeMicroseconds(700);
    leg3.writeMicroseconds(800);
}

void Command::retractLegs() {
    leg1.writeMicroseconds(2300);
    leg2.writeMicroseconds(2300);
    leg3.writeMicroseconds(2350);
}

void Command::setLedColor(bool red, bool green, bool blue) {
    // Helper: for common anode, LOW = ON, HIGH = OFF
    digitalWrite(RED_PIN, red ? LOW : HIGH);
    digitalWrite(GREEN_PIN, green ? LOW : HIGH);
    digitalWrite(BLUE_PIN, blue ? LOW : HIGH);
}