#include "Command.hpp"

void Command::commandGimbal(int angleX, int angleY) {
    /*
    Commands the TVC servos at 250Hz such that the gimbal reaches the specified angles.
    THE ANGLES GIVEN ARE NOT THE SERVO ANGLES.
    angleX: Desired angle for the X-axis (in degrees).
    angleY: Desired angle for the Y-axis (in degrees).
    */
}

void Command::commandMotors(int throttleMotor1, int throttleMotor2) {
    /*
    Commands the ESCs for the two BLDC motors.
    throttleMotor1: Throttle for motor 1 (top motor) (0-100%).
    throttleMotor2: Throttle for motor 2 (bottom motor) (0-100%).
    */
}

void Command::commandLegs(bool legsUp) {
    // Implementation to command landing legs position
}