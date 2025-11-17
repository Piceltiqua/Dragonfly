#include "IMU.hpp"

void IMU::setup(int imu_freq) {
  Serial4.begin(115200);
  bno08x_.begin_UART(&Serial4);

  bno08x_.enableReport(SH2_LINEAR_ACCELERATION, 1000000 / imu_freq);
  bno08x_.enableReport(SH2_ROTATION_VECTOR, 1000000 / imu_freq);
  bno08x_.enableReport(SH2_GYROSCOPE_CALIBRATED, 1000000 / imu_freq);
}

void IMU::read() {
  t0 = micros();
  acc = false;
  gyro = false;
  quat = false;

  // Try to read events until we've got one of each or no more data
  while (bno08x_.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        if (!acc) {
          imuAcc_.ax = -sensorValue.un.linearAcceleration.y;
          imuAcc_.ay = sensorValue.un.linearAcceleration.z;
          imuAcc_.az = -sensorValue.un.linearAcceleration.x;
          acc = true;
        }
        break;

      case SH2_ROTATION_VECTOR:
        if (!quat) {
          qIMU0 = sensorValue.un.rotationVector.real;
          qIMU1 = sensorValue.un.rotationVector.i;
          qIMU2 = sensorValue.un.rotationVector.j;
          qIMU3 = sensorValue.un.rotationVector.k;

          transformQuaternion_IMU_to_CAD(qIMU0, qIMU1, qIMU2, qIMU3,
                                         attitude_.qw, attitude_.qi, attitude_.qj, attitude_.qk);

          quat = true;
        }
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        if (!gyro) {
          attitude_.wx = -sensorValue.un.gyroscope.y;
          attitude_.wy = sensorValue.un.gyroscope.z;
          attitude_.wz = -sensorValue.un.gyroscope.x;
          gyro = true;
        }
        break;

      default:
        break;
    }

    // Quit if all data is collected or if timeout is reached
    if ((acc && gyro && quat) || micros() - t0 > IMU_TIMEOUT) {
      break;
    };
  }
}

void IMU::transformQuaternion_IMU_to_CAD(float w_in, float x_in, float y_in, float z_in,
                                         float &w_out, float &x_out, float &y_out, float &z_out) {
  // Normalize input quaternion (safety)
    float norm = std::sqrt(w_in*w_in + x_in*x_in + y_in*y_in + z_in*z_in);
    if (norm < 1e-12f) {
        // fallback to identity
        w_out = 1.0f; x_out = 0.0f; y_out = 0.0f; z_out = 0.0f;
        return;
    }
    float qw = w_in / norm;
    float qx = x_in / norm;
    float qy = y_in / norm;
    float qz = z_in / norm;

    // Build Eigen quaternion (w, x, y, z)
    Eigen::Quaternionf q_imu(qw, qx, qy, qz);
    q_imu.normalize();

    // Rotation matrix from IMU quaternion
    Eigen::Matrix3f R_imu = q_imu.toRotationMatrix();

    // Mapping matrix M such that: v_imu = M * v_cad
    // Given your mapping:
    //   x_imu = -z_cad
    //   y_imu = -x_cad
    //   z_imu =  y_cad
    Eigen::Matrix3f M;
    M <<  0.0f,  0.0f, -1.0f,
         -1.0f,  0.0f,  0.0f,
          0.0f,  1.0f,  0.0f;

    // Compute R_cad = M^T * R_imu * M  (M is orthonormal so M^T == M.inverse())
    Eigen::Matrix3f R_cad = M.transpose() * R_imu * M;

    // Convert back to quaternion
    Eigen::Quaternionf q_cad(R_cad);
    q_cad.normalize();

    // Return components (w, x, y, z)
    w_out = q_cad.w();
    x_out = q_cad.x();
    y_out = q_cad.y();
    z_out = q_cad.z();
}