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
    imu.setup(IMU_FREQ_HZ);
    gnss.setup();
    barometer.setup();
}