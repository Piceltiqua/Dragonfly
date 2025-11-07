#pragma once

struct PosVel {
    float posN = 0.0f;
    float posE = 0.0f;
    float posD = 0.0f;
    float velN = 0.0f;
    float velE = 0.0f;
    float velD = 0.0f;
};

struct Attitude {
    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    float wx = 0.0f;
    float wy = 0.0f;
    float wz = 0.0f;
};

struct IMUAcceleration {
    float accX = 0.0f;
    float accY = 0.0f;
    float accZ = 0.0f;
    float accN = 0.0f;
    float accE = 0.0f;
    float accD = 0.0f;
};

struct GNSSData {
    double latGNSS = 0.0;
    double lonGNSS = 0.0;
    double altGNSS = 0.0;
    double lat0 = 0.0;
    double lon0 = 0.0;
    double alt0 = 0.0;
    float velNGNSS = 0.0f;
    float velEGNSS = 0.0f;
    float velDGNSS = 0.0f;
    float horAcc = 0.0f;
    float vertAcc = 0.0f;
    int numSV = 0;
    int fixType = 0;
};

struct BarometerData {
    float altBaro = 0.0f;
};

struct BatteryStatus {
    float currentDraw = 0.0f;
    float currentConsumed = 0.0f;
    float batteryVoltage = 0.0f;
    float batteryLevel = 0.0f;
};

struct ActuatorCommands {
    float motor1Throttle = 0.0f;
    float motor2Throttle = 0.0f;
    bool  legsPosition = true; // false: retracted, true: deployed
    float servoXAngle = 0.0f;
    float servoYAngle = 0.0f;
};