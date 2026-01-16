#ifndef FC_H
#define FC_H

#include <SD.h>
#include <SPI.h>

#include <cmath>

#include "Battery.hpp"
#include "Command.hpp"
#include "GNSS.hpp"
#include "IMU.hpp"
#include "LQR_attitude.hpp"
#include "LQR_position.hpp"
#include "Logging.hpp"
#include "RingBuf.h"
#include "Roll_control.hpp"
#include "SdFat.h"
#include "Utils.hpp"
#include "Waypoints.hpp"

// #include "AttitudeController.hpp"
// #include "PositionController.hpp"

#define IMU_FREQ_HZ 250.0
#define TELEMETRY_FREQ_HZ 15.0

#define IMU_PERIOD_US (1000000.0 / IMU_FREQ_HZ)
#define TELEMETRY_PERIOD_US (1000000.0 / TELEMETRY_FREQ_HZ)

#define MAX_FRAME_BUFFER 2048
// Use some RAM to increase the size of the UART TX buffer, ensures that the telemetry isn't blocking
inline uint8_t extra_tx_mem[128];  // Default size of the buffer is 63 bytes, adding this leaves space (64+128) for the telemetry packets (which are around 122 bytes).

// Logging buffer sizing: tune these to available RAM and required safety.
// RING_BUF_SECTORS * 512 = ring buffer bytes.
// 8KB buffer is sufficient when draining systematically on every writeBufferToSD() call
#define RING_BUF_SECTORS 40  // 40 * 512 = 20480 bytes ring buffer (sufficient with aggressive draining)
#define RING_BUF_BYTES (512U * RING_BUF_SECTORS)

// Maximum file size to preallocate (optional). Tune to expected run time.
#define LOG_FILE_SIZE (256UL * 1024UL * 1024UL)  // 256 MiB default -> 5.8min at 385kB/s

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
    void smooth_imuread(float& wx, float& wy, float& wz);
    void updateLedColorForRTKFix();

    Attitude attitude;
    AttitudeAngle attitudeAngle;
    IMUAcceleration imuAcc;
    GNSSData gnssData;
    BatteryStatus batteryStatus;
    ActuatorCommands actuators;
    PosCtrlOutput attitudeSetpoint;
    PosCtrlSetpoint positionSetpoint;

    IMU imu;
    GNSS gnss;
    Command command;
    Battery battery;
    AttitudeController attCtrl;
    PositionController posCtrl;
    RollControl rollCtrl;
    WaypointManager waypointManager;

    elapsedMicros IMUTimer;
    bool gnssReading;
    elapsedMicros telemTimer;
    float flightStartTime = 0.0f;
    float flightTimeSeconds = 0.0f;  // Used for countdown before flight and during flight
    float lastGNSStime = 0.0f;       // Time since last GNSS reading in us, used for position controller

    float moment_arm_legs_down = 0.140f;
    float moment_arm_legs_up = 0.180f;

    int deltaTimingRoll = 0;

    bool RollControlled     = false;
    bool AttitudeControlled = false;
    bool PositionControlled = false;
    bool InFlight           = false;

    // Telemetry constants
    bool isRecording = false;
    bool inFrame = false;
    bool escapeNext = false;
    uint8_t frameBuf[MAX_FRAME_BUFFER];
    size_t frameBufLen = 0;

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

    // SD write functions
    void handleRecordingMessage(uint8_t rec);
    void initializeSD();
    void writeHeader();
    void writeToRingBuffer();
    void writeBufferToSD();

    FsFile dataFile;
    // RingBuf for FsFile with a 512-byte sector size and RING_BUF_SECTORS sectors
    RingBuf<FsFile, 512 * RING_BUF_SECTORS> rb;

    // Stats / config
    uint32_t droppedLines = 0;
    uint32_t writeSlowCount = 0;
    uint32_t writeVerySlowCount = 0;

    uint32_t write_start = 0;
    bool sdInitialized = false;
    String filename;
    uint32_t tPreviousFlush = 0;
};

#endif