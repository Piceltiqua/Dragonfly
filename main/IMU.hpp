#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO08x.h>

#include "Utils.hpp"

#define IMU_TIMEOUT 6000

class IMU {
public:
    IMU(IMUAcceleration& imuAcc, Attitude& attitude)
        : imuAcc_(imuAcc),
          attitude_(attitude) {}

    void setup(int imu_freq);
    void read();
    void imuEnuToCadNedQuat();
    void imuAccToNED(float ax_IMU, float ay_IMU, float az_IMU,
                     float& ax_NED, float& ay_NED, float& az_NED);
    void pushImuSample();

private:
    Adafruit_BNO08x bno08x_{Serial4};
    sh2_SensorValue_t sensorValue;

    IMUAcceleration& imuAcc_;
    Attitude& attitude_;

    uint32_t t0;
    bool acc, gyro, quat;
    float qIMU0, qIMU1, qIMU2, qIMU3;

    Eigen::Quaternionf q_imu_to_enu = Eigen::Quaternionf(1, 0, 0, 0);  // Stores the raw reading from IMU.
    Eigen::Quaternionf q_cad_to_ned = Eigen::Quaternionf(1, 0, 0, 0);  // Stores the transformed quaternion that maps CAD to NED.
};

#endif