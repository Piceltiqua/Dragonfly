#ifndef FC_H
#define FC_H

#include "IMU.hpp"
#include "GNSS.hpp"
#include "Barometer.hpp"
//#include "UKF.hpp"
//#include "AttitudeController.hpp"
//#include "PositionController.hpp"

#include "Utils.hpp"

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
    void readSensors();
    //void attitudeUpdate();
    //void positionUpdate();
    //void telemetryUpdate();

private:
    void printSensors();

    IMU imu;
    GNSS gnss;
    Barometer barometer;

    //UKF ukf;
    //AttitudeController attCtrl;
    //PositionController posCtrl;

    PosVel posvel;
    Attitude attitude;
    IMUAcceleration imuAcc;
    GNSSData gnssData;
    BarometerData baroData;
    BatteryStatus battery;
    ActuatorCommands actuators;
};

#endif