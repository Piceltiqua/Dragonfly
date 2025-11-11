#include "FC.hpp"

#define IMU_FREQ_HZ       250
#define GNSS_FREQ_HZ      10
#define TELEMETRY_FREQ_HZ 25

#define IMU_PERIOD_US       (1000000 / IMU_FREQ_HZ)
#define GNSS_PERIOD_US      (1000000 / GNSS_FREQ_HZ)
#define TELEMETRY_PERIOD_US (1000000 / TELEMETRY_FREQ_HZ)

FlightController FC;

elapsedMicros IMUTimer;
elapsedMicros GNSSTimer;
elapsedMicros telemTimer;

void setup() {
    FC.setup(IMU_FREQ_HZ);
}

void loop() {
    FC.readSensors();
}