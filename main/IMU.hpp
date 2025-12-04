#ifndef IMU_H
#define IMU_H

#include "Utils.hpp"
#include <Adafruit_BNO08x.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define IMU_TIMEOUT 6000

class IMU {
public:
  IMU(IMUAcceleration &imuAcc, Attitude &attitude)
    : imuAcc_(imuAcc),
      attitude_(attitude) {}

  void setup(int imu_freq);
  void read();
  void transformQuaternion_IMU_to_CAD(float w_in, float x_in, float y_in, float z_in,
                                      float &w_out, float &x_out, float &y_out, float &z_out);

    private : Adafruit_BNO08x bno08x_{ Serial4 };
  sh2_SensorValue_t sensorValue;

  IMUAcceleration &imuAcc_;
  Attitude &attitude_;

  uint32_t t0;
  bool acc, gyro, quat;
  float qIMU0, qIMU1, qIMU2, qIMU3;
};

#endif