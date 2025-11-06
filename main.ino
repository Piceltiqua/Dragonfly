// ========== LOOP FREQUENCIES ==========
#define IMU_FREQ_HZ       250
#define GNSS_FREQ_HZ      10
#define TELEMETRY_FREQ_HZ 25

#define IMU_PERIOD_US       (1000000 / IMU_FREQ_HZ)
#define GNSS_PERIOD_US      (1000000 / GNSS_FREQ_HZ)
#define TELEMETRY_PERIOD_US (1000000 / TELEMETRY_FREQ_HZ)

elapsedMicros IMUTimer;
elapsedMicros GNSSTimer;
elapsedMicros telemTimer;

void setup() {
    Serial.begin(115200);
}

void loop() {
  // ---------- High-rate IMU loop ----------
  if (IMUTimer >= IMU_PERIOD_US) {
    IMUTimer -= IMU_PERIOD_US;   // maintain precise timing

    // Read IMU data

    // UKF predict

    // Compute attitude control (PID)

    // Send commands to actuators

    // Update battery level
  }

  // ---------- Low-rate GNSS loop ----------
  if (GNSSTimer >= GNSS_PERIOD_US) {
    GNSSTimer -= GNSS_PERIOD_US;

    // Read GNSS

    // UKF update step

    // Position control

    // Adjust attitude targets accordingly
  }

  // ----------  Telemetry loop ----------
  if (telemTimer >= TELEMETRY_PERIOD_US) {
    telemTimer -= TELEMETRY_PERIOD_US;

    // Send telemetry data
    // Save to SD card
  }

  // ---------- Optional: background tasks ----------
  // e.g., log data, monitor battery, handle RC inputs
}