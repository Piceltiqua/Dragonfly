#include "IMU.hpp"

IMU::setup(int IMU_FREQ_HZ) {
    Serial4.begin(115200);
    bno08x_.begin_UART(&Serial4);

    bno08x_.enableReport(SH2_LINEAR_ACCELERATION,   1000000 / IMU_FREQ_HZ);
    bno08x_.enableReport(SH2_ROTATION_VECTOR,       1000000 / IMU_FREQ_HZ);
    bno08x_.enableReport(SH2_GYROSCOPE_CALIBRATED,  1000000 / IMU_FREQ_HZ);
}

IMU::read() {
    while (bno08x_.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_LINEAR_ACCELERATION:
                imuAcc_.ax = sensorValue.un.linear_acceleration.x;
                imuAcc_.ay = sensorValue.un.linear_acceleration.y;
                imuAcc_.az = sensorValue.un.linear_acceleration.z;
                break;
            case SH2_ROTATION_VECTOR: {
                attitude_.q0 = sensorValue.un.rotation_vector.real;
                attitude_.q1 = sensorValue.un.rotation_vector.i;
                attitude_.q2 = sensorValue.un.rotation_vector.j;
                attitude_.q3 = sensorValue.un.rotation_vector.k;
                break;
            }
            case SH2_GYROSCOPE_CALIBRATED:
                attitude_.wx = sensorValue.un.gyroscope.x;
                attitude_.wy = sensorValue.un.gyroscope.y;
                attitude_.wz = sensorValue.un.gyroscope.z;
                break;
            default:
                break;
        }
    }
}