#ifndef FC_H
#define FC_H

#include "Command.hpp"
#include "GNSS.hpp"
#include "IMU.hpp"
#include "UKF.hpp"
#include "Utils.hpp"
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
    // PositionController posCtrl;

    elapsedMicros IMUTimer;
    bool gnssReading;
    elapsedMicros telemTimer;

    // Telemetry constants
    bool isRecording = false;
    bool inFrame = false;
    bool escapeNext = false;
    uint8_t frameBuf[MAX_FRAME_BUFFER];
    size_t frameBufLen = 0;

    // Telemetry functions
    void processIncomingByte(uint8_t b);  // Read incoming data
    void sendTelemetry();

    uint16_t crc16_ccitt(const uint8_t* data, size_t len);
    void writeEscapedByte(uint8_t b);
    void sendRawFramed(const uint8_t* unescaped, size_t unescapedLen);
    void sendPayloadWithCrc(const uint8_t* payloadWithCrc, size_t payloadLen);
    void buildPackedPayload(uint8_t* buf, size_t& outLen);
    void executeCommandFromPayload(const uint8_t* payload, size_t payloadLen);
    void processCompleteUnescapedFrame(const uint8_t* buf, size_t len);
    bool trySendPayloadWithCrc(const uint8_t* payloadWithCrc, size_t payloadLen);
};

#endif