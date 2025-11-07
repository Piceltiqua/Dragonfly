#include "FlightController.hpp"

FlightController::FlightController()
    : imu(imuAcc, attitude),
      gnss(gnssData),
      ukf(posvel, attitude, imuAcc, gnssData, barometerData),
      barometer(barometerData),
      attitudeCtrl(attitude, actuatorCmds),
      positionCtrl(posvel, actuatorCmds)
{}

void FlightController::setup() {
    imu.setup();
    gnss.setup();
    barometer.setup();
}