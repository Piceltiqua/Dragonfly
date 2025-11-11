#ifndef IMU_H
#define IMU_H

#include "Utils.hpp"
#include <Adafruit_BNO08x.h>

class IMU {
public:
    IMU(IMUAcceleration &imuAcc, Attitude &attitude):
    imuAcc_(imuAcc),
    position_(position)
    {}

    void setup(int IMU_FREQ_HZ);
    void read();

private:
    Adafruit_BNO08x bno08x_{Serial4};
    sh2_SensorValue_t sensorValue;
    
    IMUAcceleration &imuAcc_;
    Attitude &attitude_;
}
#endif