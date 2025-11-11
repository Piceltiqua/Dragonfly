#include "IMU.hpp"

void IMU::setup(int imu_freq) {
    Serial4.begin(115200);
    bno08x_.begin_UART(&Serial4);

    bno08x_.enableReport(SH2_LINEAR_ACCELERATION,   1000000 / imu_freq);
    bno08x_.enableReport(SH2_ROTATION_VECTOR,       1000000 / imu_freq);
    bno08x_.enableReport(SH2_GYROSCOPE_CALIBRATED,  1000000 / imu_freq);
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
                    imuAcc_.ax = sensorValue.un.linearAcceleration.x;
                    imuAcc_.ay = sensorValue.un.linearAcceleration.y;
                    imuAcc_.az = sensorValue.un.linearAcceleration.z;
                    acc = true;
                }
                break;

            case SH2_ROTATION_VECTOR:
                if (!quat) {
                    attitude_.q0 = sensorValue.un.rotationVector.real;
                    attitude_.q1 = sensorValue.un.rotationVector.i;
                    attitude_.q2 = sensorValue.un.rotationVector.j;
                    attitude_.q3 = sensorValue.un.rotationVector.k;
                    quat = true;
                }
                break;

            case SH2_GYROSCOPE_CALIBRATED:
                if (!gyro) {
                    attitude_.wx = sensorValue.un.gyroscope.x;
                    attitude_.wy = sensorValue.un.gyroscope.y;
                    attitude_.wz = sensorValue.un.gyroscope.z;
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