#ifndef FC_H
#define FC_H

#include "Command.hpp"
#include "GNSS.hpp"
#include "IMU.hpp"
#include "UKF.hpp"
#include "Utils.hpp"
#include "LQR_attitude.hpp"
#include <Eigen/LU>
#include <cmath>
#include <algorithm>

// #include "AttitudeController.hpp"
// #include "PositionController.hpp"

#define IMU_FREQ_HZ 250.0
#define TELEMETRY_FREQ_HZ 20.0

#define IMU_PERIOD_US (1000000.0 / IMU_FREQ_HZ)
#define TELEMETRY_PERIOD_US (1000000.0 / TELEMETRY_FREQ_HZ)

#define MAX_FRAME_BUFFER 2048

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

private:
    void printState();
    // void printSensors();

    PosVel posvel;
    Attitude attitude;
    IMUAcceleration imuAcc;
    GNSSData gnssData;
    BatteryStatus battery;
    ActuatorCommands actuators;
    
    IMU imu;
    GNSS gnss;
    Command command;
    UKF ukf;
    // AttitudeController attCtrl;
    LQR_attitude attitudeCtrl;
    // PositionController posCtrl;

    elapsedMicros IMUTimer;
    bool gnssReading;
    elapsedMicros telemTimer;
    //Controleur constants
    Eigen::Matrix<float, 2, 4> K_lqr =
        (Eigen::Matrix<float, 2, 4>() << 0.5345f, -0.0000f, 0.2484f, -0.0000f,
                                   -0.0000f, 0.5345f, -0.0000f, 0.2484f)
            .finished();
    Attitude_angle lqr_att;
    Attitude_angle current_attitude;
    rotationspeed lqr_rates;
    AttitudeSetpoint lqr_sp;
    ControlOutput_attitude lqr_out;
    // Telemetry constants
    bool isRecording = false;
    bool inFrame = false;
    bool escapeNext = false;
    uint8_t frameBuf[MAX_FRAME_BUFFER];
    size_t frameBufLen = 0;
    //conversion quaternion to euler angles
    void quaternionToEuler(float qw, float qi, float qj, float qk,
                           float &roll, float &pitch, float &yaw);
    // Telemetry functions
    void processIncomingByte(uint8_t b);  // Read incoming data
    void sendTelemetry();
    void writeEscapedByte(uint8_t b);
    void sendRawFramed(const uint8_t* unescaped, size_t unescapedLen);

    uint16_t crc16_ccitt(const uint8_t* data, size_t len);
    void buildPackedPayload(uint8_t* buf, size_t& outLen);
    void executeCommandFromPayload(const uint8_t* payload, size_t payloadLen);
    void processCompleteUnescapedFrame(const uint8_t* buf, size_t len);
    bool trySendPayloadWithCrc(const uint8_t* payloadWithCrc, size_t payloadLen);
    void AttitudeHold();
};

#endif