#pragma once
#include "IMU.hpp"
#include "GNSS.hpp"
#include "UKF.hpp"
#include "Barometer.hpp"
#include "AttitudeController.hpp"
#include "PositionController.hpp"

#include "utils.hpp"

enum class FCState {
    NoFix,
    Idle,
    TakingOff,
    AttitudeHold,
    PositionHold,
    WaypointGuidance,
    Landing
};

class FlightController {
public:
    FlightController();

    void setup(int IMU_FREQ_HZ);
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

    PosVel posvel;
    Attitude attitude;
    IMUAcceleration imuAcc;
    GNSSData gnssData;
    BarometerData baroData;
    BatteryStatus battery;
    ActuatorCommands actuators;
};

