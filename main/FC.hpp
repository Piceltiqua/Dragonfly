#ifndef FC_H
#define FC_H

#include "IMU.hpp"
#include "GNSS.hpp"
#include "Barometer.hpp"
//#include "UKF.hpp"
//#include "AttitudeController.hpp"
//#include "PositionController.hpp"

#define IMU_FREQ_HZ      250
#define GNSS_FREQ_HZ      10
#define TELEMETRY_FREQ_HZ 25

#define IMU_PERIOD_US             (1000000 / IMU_FREQ_HZ)
#define GNSS_PERIOD_US           (1000000 / GNSS_FREQ_HZ)
#define TELEMETRY_PERIOD_US (1000000 / TELEMETRY_FREQ_HZ)

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

    void setup();
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

    elapsedMicros IMUTimer;
    bool gnssReading;
    elapsedMicros telemTimer;

    PosVel posvel;
    Attitude attitude;
    IMUAcceleration imuAcc;
    GNSSData gnssData;
    BarometerData baroData;
    BatteryStatus battery;
    ActuatorCommands actuators;
};

#endif