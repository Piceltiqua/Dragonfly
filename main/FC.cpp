#include "FlightController.hpp"

FlightController::FlightController()
    : imu(imuAcc, attitude),
      gnss(gnssData),
      ukf(posvel, attitude, imuAcc, gnssData, barometerData),
      barometer(barometerData),
      attitudeCtrl(attitude, actuatorCmds),
      positionCtrl(posvel, actuatorCmds)
{}

void FlightController::setup(int IMU_FREQ_HZ) {
    Serial.begin(115200);
    imu.setup(IMU_FREQ_HZ);
    gnss.setup();
    barometer.setup();
}

void FlightController::readSensors() {
      // ---------- High-rate IMU loop ----------
    if (IMUTimer >= IMU_PERIOD_US) {
        IMUTimer -= IMU_PERIOD_US;
        IMU.read();
        printSensors();
      
    // Update battery level
    }

    if (GNSSTimer >= GNSS_PERIOD_US) {
        GNSSTimer -= GNSS_PERIOD_US;
        GNSS.read();
        Barometer.read();
    }

    // ---------- Telemetry loop ----------
    // if (telemTimer >= TELEMETRY_PERIOD_US) {
    //     telemTimer -= TELEMETRY_PERIOD_US;
    //     FC.telemetryUpdate();
    // }
}

void FlightController::printSensors() {
    Serial.print(attitude.q0); Serial.print(", ");
    Serial.print(attitude.q1); Serial.print(", ");
    Serial.print(attitude.q2); Serial.print(", ");
    Serial.print(attitude.q3); Serial.print(", ");
    Serial.print(attitude.wx); Serial.print(", ");
    Serial.print(attitude.wy); Serial.print(", ");
    Serial.print(attitude.wz); Serial.print(", ");
    Serial.print(imuAcc.accX); Serial.print(", ");
    Serial.print(imuAcc.accY); Serial.print(", ");
    Serial.print(imuAcc.accZ); Serial.print(", ");
    Serial.print(gnssData.latGNSS); Serial.print(", ");
    Serial.print(gnssData.lonGNSS); Serial.print(", ");
    Serial.print(gnssData.altGNSS); Serial.print(", ");
    Serial.print(gnssData.velNGNSS); Serial.print(", ");
    Serial.print(gnssData.velEGNSS); Serial.print(", ");
    Serial.print(gnssData.velDGNSS); Serial.print(", ");
    Serial.print(gnssData.horAcc); Serial.print(", ");
    Serial.print(gnssData.vertAcc); Serial.print(", ");
    Serial.print(gnssData.numSV); Serial.print(", ");
    Serial.print(gnssData.fixType); Serial.print(", ");
    Serial.print(baroData.altBaro); Serial.println();
}