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
                    float ax_IMU = sensorValue.un.linearAcceleration.x;
                    float ay_IMU = sensorValue.un.linearAcceleration.y;
                    float az_IMU = sensorValue.un.linearAcceleration.z;

                    imuAccToNED(ax_IMU, ay_IMU, az_IMU, imuAcc_.ax_NED, imuAcc_.ay_NED, imuAcc_.az_NED);

                    acc = true;
                }
                break;

            case SH2_ROTATION_VECTOR:
                if (!quat) {
                    q_imu_to_enu.w() = sensorValue.un.rotationVector.real;
                    q_imu_to_enu.x() = sensorValue.un.rotationVector.i;
                    q_imu_to_enu.y() = sensorValue.un.rotationVector.j;
                    q_imu_to_enu.z() = sensorValue.un.rotationVector.k;

                    imuEnuToCadNedQuat();

                    // Store the output quaternion in the attitude struct.
                    attitude_.qw = q_cad_to_ned.w();
                    attitude_.qi = q_cad_to_ned.x();
                    attitude_.qj = q_cad_to_ned.y();
                    attitude_.qk = q_cad_to_ned.z();

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

void IMU::imuEnuToCadNedQuat() {
    // Ensure input normalized to avoid numerical issues
    q_imu_to_enu = q_imu_to_enu.normalized();

    // Combine the rotations: q_ned_cad = q_ned_enu * q_enu_imu * q_imu_cad
    q_cad_to_ned = q_enu_to_ned * q_imu_to_enu * q_cad_to_imu;

    // Normalize output quaternion
    q_cad_to_ned = q_cad_to_ned.normalized();
}

void IMU::imuAccToNED(float ax_IMU, float ay_IMU, float az_IMU,
                      float& ax_NED, float& ay_NED, float& az_NED){
    // Converts the acceleration from the IMU frame to the NED frame, to express the acceleration in world frame (NED)

    // Acceleration vector in the frame of the IMU
    Eigen::Vector3f acc_imu(ax_IMU, ay_IMU, az_IMU);

    // Acceleration vector in the CAD frame, computed from the fixed quaternion that maps CAD to IMU.
    Eigen::Vector3f acc_cad = q_cad_to_imu.inverse() * acc_imu;

    // Rotate to from CAD frame to NED using quaternion-vector multiplication
    Eigen::Vector3f acc_ned = q_cad_to_ned * acc_cad;

    ax_NED = acc_ned.x();
    ay_NED = acc_ned.y();
    az_NED = acc_ned.z();
}