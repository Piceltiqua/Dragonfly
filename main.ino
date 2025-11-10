#include "FlightController.hpp"

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
  // ---------- High-rate IMU loop ----------
  if (IMUTimer >= IMU_PERIOD_US) {
    IMUTimer -= IMU_PERIOD_US;
    FC.attitudeUpdate();
      
    // Update battery level
  }

  // ---------- Low-rate GNSS loop ----------
  if (GNSSTimer >= GNSS_PERIOD_US) {
    GNSSTimer -= GNSS_PERIOD_US;
    FC.positionUpdate();
  }

  // ---------- Telemetry loop ----------
  if (telemTimer >= TELEMETRY_PERIOD_US) {
    telemTimer -= TELEMETRY_PERIOD_US;
    FC.telemetryUpdate();
  }
}