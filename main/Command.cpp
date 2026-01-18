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

    extendLegs();

    motor1.writeMicroseconds(1100);
    motor2.writeMicroseconds(1100);
}

void Command::commandGimbal(float newGimbalAngleX, float newGimbalAngleY) {
    /*
    Commands the TVC servos at 50Hz such that the gimbal reaches the specified angles.
    THE ANGLES GIVEN ARE NOT THE SERVO ANGLES.
    newGimbalAngleX: Desired angle for the X-axis (in degrees).
    newGimbalAngleY: Desired angle for the Y-axis (in degrees).
    */

    if (newGimbalAngleX > MAX_GIMBAL_ANGLE) {
        newGimbalAngleX = MAX_GIMBAL_ANGLE;
    }
    if (newGimbalAngleX < -MAX_GIMBAL_ANGLE) {
        newGimbalAngleX = -MAX_GIMBAL_ANGLE;
    }
    if (newGimbalAngleY > MAX_GIMBAL_ANGLE) {
        newGimbalAngleY = MAX_GIMBAL_ANGLE;
    }
    if (newGimbalAngleY < -MAX_GIMBAL_ANGLE) {
        newGimbalAngleY = -MAX_GIMBAL_ANGLE;
    }

    // If no change in angles, return
    if ((abs(newGimbalAngleX - currentGimbalAngleX) < 0.05f) && (abs(newGimbalAngleY - currentGimbalAngleY) < 0.05f)) {
        return;
    }

    // If increasing angle motion
    if (newGimbalAngleX >= currentGimbalAngleX) {
        actuatorCmds_.servoXAngle = -2.034e-2 * pow(newGimbalAngleX, 3) + 3.762e-2 * pow(newGimbalAngleX, 2) - 5.142 * newGimbalAngleX;
    }
    if (newGimbalAngleY >= currentGimbalAngleY) {
        actuatorCmds_.servoYAngle = -3.339e-2 * pow(newGimbalAngleY, 2) - 5.364 * newGimbalAngleY + 13;
    }
    // If decreasing angle motion
    if (newGimbalAngleX < currentGimbalAngleX) {
        actuatorCmds_.servoXAngle = -8.894e-2 * pow(newGimbalAngleX, 2) - 5.128 * newGimbalAngleX - 3;
    }
    if (newGimbalAngleY < currentGimbalAngleY) {
        actuatorCmds_.servoYAngle = -1.267e-2 * pow(newGimbalAngleY, 3) - 5.972e-2 * pow(newGimbalAngleY, 2) - 4.834 * newGimbalAngleY + 6;
    }
    currentGimbalAngleX = newGimbalAngleX;
    currentGimbalAngleY = newGimbalAngleY;

    uint16_t timingX = map(actuatorCmds_.servoXAngle, -60.0f, 60.0f, 900, 2100);
    uint16_t timingY = map(actuatorCmds_.servoYAngle, -60.0f, 60.0f, 900, 2100);

    gimbalX.writeMicroseconds(timingX);
    gimbalY.writeMicroseconds(timingY);

    // Serial.print("Gimbal X angle (deg): ");
    // Serial.println(newGimbalAngleX);
    // Serial.print("Gimbal Y angle (deg): ");
    // Serial.println(newGimbalAngleY);
    // Serial.print("Servo X angle (deg): ");
    // Serial.println(actuatorCmds_.servoXAngle);
    // Serial.print("Servo Y angle (deg): ");
    // Serial.println(actuatorCmds_.servoYAngle);
}

void Command::commandMotorsThrust(float thrustMotor, float rollTimingOffset) {
    /*
    Commands the ESCs for the two BLDC motors.
    throttleMotor1: Thrust command for motor 1 (top motor) (0-2060g).
    throttleMotor2: Thrust command for motor 2 (bottom motor) (0-2060g).
    */

    if (thrustMotor > 2060.0) {
        thrustMotor = 2060.0;
    }
    if (thrustMotor < 0.0) {
        thrustMotor = 0.0;
    }

    timingMotor1 = thrustToTiming(thrustMotor) - rollTimingOffset;
    timingMotor2 = thrustToTiming(thrustMotor) + rollTimingOffset;

    motor1.writeMicroseconds(timingMotor1);
    motor2.writeMicroseconds(timingMotor2);

    // Serial.print("Motor thrust (g): ");
    // Serial.println(thrustMotor);
    // Serial.print("Motor 1 timing (us): ");
    // Serial.println(timingMotor1);
    // Serial.print("Motor 2 timing (us): ");
    // Serial.println(timingMotor2);
    // Serial.print("Roll timing offset (us): ");
    // Serial.println(rollTimingOffset);
}

void Command::adjustMotorThrustForBatteryVoltage(int16_t voltage_mV) {
    if (voltage_mV >= 12377) {
        actuatorCmds_.thrustBatteryCoefficient = 2.98e-4 * voltage_mV - 2.6;
    } else {
        actuatorCmds_.thrustBatteryCoefficient = 1.39e-4 * voltage_mV - 0.632;
    }
}

int Command::thrustToTiming(float thrust_gram) {
    float timing_float = 1100.0;

    thrust_gram = thrust_gram / actuatorCmds_.thrustBatteryCoefficient;

    if (thrust_gram >= 0 && thrust_gram < 400.0) {
        timing_float = 4.44e-6 * pow(thrust_gram, 3) - 3.65e-3 * pow(thrust_gram, 2) + 1.43 * thrust_gram + 1120.0;
    } else if (thrust_gram >= 400.0 && thrust_gram <= 2060.0) {
        timing_float = -2.12e-5 * pow(thrust_gram, 2) + 3.69e-1 * thrust_gram + 1250;
    }

    int timing_int = static_cast<int>(timing_float);

    if (timing_int > 1940) {
        timing_int = 1940;
    }
    if (timing_int < 1100) {
        timing_int = 1100;
    }

    return timing_int;
}

void Command::extendLegs() {
    actuatorCmds_.legsPosition = LEGS_DEPLOYED;
    leg1.writeMicroseconds(650);
    leg2.writeMicroseconds(700);
    leg3.writeMicroseconds(800);
}

void Command::retractLegs() {
    actuatorCmds_.legsPosition = LEGS_RETRACTED;
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