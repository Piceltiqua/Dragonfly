#ifndef IMU_H
#define IMU_H

#include "Utils.hpp"
#include <Adafruit_BNO08x.h>

#define IMU_TIMEOUT 1000

class IMU {
public:
    IMU(IMUAcceleration &imuAcc, Attitude &attitude):
    imuAcc_(imuAcc),
    attitude_(attitude)
    {}

    void setup(int imu_freq);
    void read();

private:
    Adafruit_BNO08x bno08x_{Serial4};
    sh2_SensorValue_t sensorValue;
    
    IMUAcceleration &imuAcc_;
    Attitude &attitude_;

    uint32_t t0;
    bool acc;
    bool gyro;
    bool quat;
};

#endif