#ifndef FC_H
#define FC_H

#include "IMU.hpp"
#include "GNSS.hpp"
#include "Barometer.hpp"
#include "UKF.hpp"
//#include "AttitudeController.hpp"
//#include "PositionController.hpp"

#define IMU_FREQ_HZ      250
#define TELEMETRY_FREQ_HZ 20

#define IMU_PERIOD_US             (1000000.0 / IMU_FREQ_HZ)
#define TELEMETRY_PERIOD_US (1000000.0 / TELEMETRY_FREQ_HZ)

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
    void sensorFusion();

private:
    void printState();
    void printSensors();

    IMU imu;
    GNSS gnss;
    Barometer barometer;

    UKF ukf;
    //AttitudeController attCtrl;
    //PositionController posCtrl;

    elapsedMicros IMUTimer;
    bool gnssReading;
    elapsedMicros telemTimer;

    PosVel posvel;
    Attitude attitude;
    IMUAcceleration imuAcc;
    GNSSData gnssData;
    BarometerData barometerData;
    BatteryStatus battery;
    ActuatorCommands actuators;
};

#endif