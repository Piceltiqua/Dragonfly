#include "FlightController.hpp"

FlightController::FlightController()
    : imu(imuAcc, attitude),
      gnss(gnssData),
      ukf(position, attitude, imuAcc, gnssData, barometerData),
      barometer(barometerData),
      attitudeCtrl(attitude, actuatorCmds),
      positionCtrl(position, actuatorCmds)
{}

void FlightController::setup() {
    imu.setup();
    gnss.setup();
    barometer.setup();
}