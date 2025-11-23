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
                    // TODO : Convert the acceleration from CAD frame to NED
                    // frame, by using the converted rotation vector quaternion
                    // (that does NED -> CAD)
                }
                break;

            case SH2_ROTATION_VECTOR:
                if (!quat) {
                    qIMU0 = sensorValue.un.rotationVector.real;
                    qIMU1 = sensorValue.un.rotationVector.i;
                    qIMU2 = sensorValue.un.rotationVector.j;
                    qIMU3 = sensorValue.un.rotationVector.k;

                    Serial.print("IMU Quat:\t");
                    Serial.print(qIMU0);
                    Serial.print("\t");
                    Serial.print(qIMU1);
                    Serial.print("\t");
                    Serial.print(qIMU2);
                    Serial.print("\t");
                    Serial.println(qIMU3);

                    imuQuatToCadQuatf(qIMU0, qIMU1, qIMU2, qIMU3,
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

void IMU::imuEnuToCadNedQuat(const Eigen::Quaternionf& q_imu, Eigen::Quaternionf& q_ned_cad) {
    // Ensure input normalized to avoid numerical issues
    Eigen::Quaternionf q_enu_imu = q_imu.normalized();

    // Fixed quaternion encoding the axis mapping between the CAD and the IMU
    const Eigen::Quaternionf q_imu_cad = Eigen::Quaternionf(-0.5, -0.5, 0.5, 0.5);  // w,x,y,z

    // Fixed quaternion that converts ENU to NED
    const Eigen::Quaternionf q_ned_enu = Eigen::Quaternionf(0, 0.7071068, 0.7071068, 0);  // w,x,y,z

    // Combine the rotations: q_ned_cad = q_ned_enu * q_enu_imu * q_imu_cad
    Eigen::Quaternionf q_cad = q_ned_enu * q_enu_imu * q_imu_cad;

    // Normalize output quaternion
    q_ned_cad = q_cad.normalized();
}

// Convenience wrapper using floats (w,x,y,z)
void IMU::imuEnuToCadNedQuatf(float qw_in, float qx_in, float qy_in, float qz_in,
                              float& qw_out, float& qx_out, float& qy_out, float& qz_out) {
    Eigen::Quaternionf q_imu(qw_in, qx_in, qy_in, qz_in);  // (w,x,y,z)
    Eigen::Quaternionf q_cad;
    imuEnuToCadNedQuat(q_imu, q_cad);
    qw_out = q_cad.w();
    qx_out = q_cad.x();
    qy_out = q_cad.y();
    qz_out = q_cad.z();
}