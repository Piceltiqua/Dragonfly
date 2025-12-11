#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO08x.h>

#include "Utils.hpp"

#define IMU_TIMEOUT 6000

struct ImuSnapshot {
    // From imuEnuToCadNedQuat()
    float q_imu_to_enu_w, q_imu_to_enu_x, q_imu_to_enu_y, q_imu_to_enu_z;  // Raw IMU quaternion
    float q_cad_to_ned_w, q_cad_to_ned_x, q_cad_to_ned_y, q_cad_to_ned_z;  // Transformed quaternion

    // From imuAccToNED()
    float acc_imu_x, acc_imu_y, acc_imu_z;  // Acceleration in IMU frame
    float acc_cad_x, acc_cad_y, acc_cad_z;  // Acceleration in CAD frame
    float acc_ned_x, acc_ned_y, acc_ned_z;  // Acceleration in NED frame

    // From pushImuSample()
    float omega_imu_x, omega_imu_y, omega_imu_z;  // Angular rates in IMU frame

    uint32_t t_us;  // Timestamp in microseconds
};

class IMU {
public:
    IMU(IMUAcceleration& imuAcc, Attitude& attitude)
        : imuAcc_(imuAcc),
          attitude_(attitude),
          last_snapshot_() {}

    void setup(int imu_freq);
    void read();
    void imuEnuToCadNedQuat();
    void imuAccToNED(float ax_IMU, float ay_IMU, float az_IMU,
                     float& ax_NED, float& ay_NED, float& az_NED);
    void pushImuSample();
    const ImuSnapshot& getLastSnapshot() const { return last_snapshot_; }

private:
    Adafruit_BNO08x bno08x_{Serial4};
    sh2_SensorValue_t sensorValue;
    IMUAcceleration& imuAcc_;
    Attitude& attitude_;

    ImuSnapshot last_snapshot_;

    uint32_t t0;
    bool acc, gyro, quat;
    float qIMU0, qIMU1, qIMU2, qIMU3;

    Eigen::Quaternionf q_imu_to_enu = Eigen::Quaternionf(1, 0, 0, 0);  // Stores the raw reading from IMU.
    Eigen::Quaternionf q_cad_to_ned = Eigen::Quaternionf(1, 0, 0, 0);  // Stores the transformed quaternion that maps CAD to NED.
};

#endif