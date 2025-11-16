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

                    attitude_.qw = 0.5 * (qIMU0 + qIMU1 + qIMU2 + qIMU3);
                    attitude_.qi = 0.5 * (qIMU1 - qIMU0 - qIMU3 + qIMU2);
                    attitude_.qj = 0.5 * (qIMU2 + qIMU3 - qIMU0 - qIMU1);
                    attitude_.qk = 0.5 * (qIMU3 - qIMU2 + qIMU1 - qIMU0);
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