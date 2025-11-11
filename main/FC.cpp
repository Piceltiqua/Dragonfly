#include "FC.hpp"

FlightController::FlightController()
    : imu(imuAcc, attitude),
      gnss(gnssData),
      barometer(baroData)
      //ukf(posvel, attitude, imuAcc, gnssData, barometerData),
      //attitudeCtrl(attitude, actuatorCmds),
      //positionCtrl(posvel, actuatorCmds)
{}

void FlightController::setup() {
    Serial.begin(115200);
    imu.setup(IMU_FREQ_HZ);
    gnss.setup();
    barometer.setup();

    delay(1000);
}

void FlightController::readSensors() {
    // Attitude loop
    if (IMUTimer >= IMU_PERIOD_US) {
        IMUTimer -= IMU_PERIOD_US;
        imu.read();
        gnssReading = gnss.read();
      
        // Update battery level
    }

    // Position loop
    if (gnssReading) {
        barometer.read();
    }

    // Telemetry loop
    if (telemTimer >= TELEMETRY_PERIOD_US) {
        telemTimer -= TELEMETRY_PERIOD_US;
        printSensors();
    }
}

void FlightController::printSensors() {
    Serial.print(millis());    Serial.print(",");
    Serial.print(attitude.q0); Serial.print(",");
    Serial.print(attitude.q1); Serial.print(",");
    Serial.print(attitude.q2); Serial.print(",");
    Serial.print(attitude.q3); Serial.print(",");
    Serial.print(attitude.wx); Serial.print(",");
    Serial.print(attitude.wy); Serial.print(",");
    Serial.print(attitude.wz); Serial.print(",");
    Serial.print(imuAcc.ax);   Serial.print(",");
    Serial.print(imuAcc.ay);   Serial.print(",");
    Serial.print(imuAcc.az);   Serial.print(",");
    Serial.print(gnssData.latGNSS);  Serial.print(",");
    Serial.print(gnssData.lonGNSS);  Serial.print(",");
    Serial.print(gnssData.altGNSS);  Serial.print(",");
    Serial.print(gnssData.posNGNSS); Serial.print(",");
    Serial.print(gnssData.posEGNSS); Serial.print(",");
    Serial.print(gnssData.posDGNSS); Serial.print(",");
    Serial.print(gnssData.velNGNSS); Serial.print(",");
    Serial.print(gnssData.velEGNSS); Serial.print(",");
    Serial.print(gnssData.velDGNSS); Serial.print(",");
    Serial.print(gnssData.horAcc);   Serial.print(",");
    Serial.print(gnssData.vertAcc);  Serial.print(",");
    Serial.print(gnssData.numSV);    Serial.print(",");
    Serial.print(gnssData.fixType);  Serial.print(",");
    Serial.print(baroData.altBaro);  Serial.println();
}