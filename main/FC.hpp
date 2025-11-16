#ifndef FC_H
#define FC_H

#include "Barometer.hpp"
#include "GNSS.hpp"
#include "IMU.hpp"
#include "Utils.hpp"
#include "Command.hpp"
// #include "UKF.hpp"
// #include "AttitudeController.hpp"
// #include "PositionController.hpp"

#define IMU_FREQ_HZ 250
#define GNSS_FREQ_HZ 10
#define TELEMETRY_FREQ_HZ 25

#define IMU_PERIOD_US (1000000 / IMU_FREQ_HZ)
#define GNSS_PERIOD_US (1000000 / GNSS_FREQ_HZ)
#define TELEMETRY_PERIOD_US (1000000 / TELEMETRY_FREQ_HZ)

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
    // void attitudeUpdate();
    // void positionUpdate();
    // void telemetryUpdate();

private:
    void printSensors();

    IMU imu;
    GNSS gnss;
    Barometer barometer;
    Command command;

    // UKF ukf;
    // AttitudeController attCtrl;
    // PositionController posCtrl;

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
};

#endif