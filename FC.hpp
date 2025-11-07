#pragma once
#include "IMU.hpp"
#include "GNSS.hpp"
#include "UKF.hpp"
#include "Barometer.hpp"
#include "AttitudeController.hpp"
#include "PositionController.hpp"

#include "utils.hpp"

class FlightController {
public:
    FlightController();

    void setup();
    void attitudeUpdate();
    void positionUpdate();
    void telemetryUpdate();

private:
    IMU imu;
    GNSS gnss;
    UKF ukf;
    Barometer barometer;
    AttitudeController attCtrl;
    PositionController posCtrl;

    Position position;
    Attitude attitude;
    IMUAcceleration imuAcc;
    GNSSData gnssData;
    BarometerData baroData;
    BatteryStatus battery;
    ActuatorCommands actuators;
};

