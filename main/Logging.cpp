#include "FC.hpp"

void FlightController::handleRecordingMessage(uint8_t rec) {
    // Command to start recording
    if (rec == 0xE6) {
        // If we are not already recording
        if (!isRecording) {
            // If SD not initialized, try to initialize it
            if (!sdInitialized) {
                initializeSD();
            }

            // If SD initialized and GNSS has a good fix, open file
            if (sdInitialized) {
                filename = gnss.getDateFilename();
                dataFile = SD.sdfs.open(filename.c_str(), O_WRITE | O_CREAT);
                if (!dataFile.isOpen()) {
                    Serial.println("Failed to open file for logging");
                    isRecording = false;
                } else {
                    // Pre-allocate to avoid FAT allocation during logging (helps reduce long pauses)
                    if (dataFile.preAllocate(LOG_FILE_SIZE)) {
                        Serial.print("Preallocated ");
                        Serial.print(LOG_FILE_SIZE);
                        Serial.print(" bytes for ");
                        Serial.println(filename);
                    } else {
                        Serial.println("Unable to preallocate this file (continuing without prealloc)");
                    }

                    // Initialize ring buffer bound to this file. This is crucial.
                    rb.begin(&dataFile);

                    // Optionally: write the header into the ring buffer so header is also logged.
                    writeHeader();

                    isRecording = true;
                    Serial.print("Opened file with name: ");
                    Serial.println(filename);
                }
            } else {
                isRecording = false;
            }
        }
    }

    // Command to stop recording
    else if (rec == 0xF1) {
        if (isRecording) {
            // Write any remaining ring buffer data to the file and truncate the pre-allocated tail.
            rb.sync();  // write remaining bytes to file (may block briefly to write last sectors)
            // Remove remaining pre-allocated space
            dataFile.truncate();
            dataFile.close();
            isRecording = false;
            Serial.println("Closed file.");
        }
    }
}

void FlightController::initializeSD() {
    // Initialize SD card (needed before attempting to open files)
    if (!SD.sdfs.begin(SdioConfig(FIFO_SDIO)))  // Use SDFat library with DMA, alternative is SD.sdfs.begin(SdioConfig(FIFO_SDIO))
    {
        Serial.println("SD.sdfs.begin() failed - SD card not found");
        sdInitialized = false;
    } else {
        Serial.println("SD initialized successfully");
        sdInitialized = true;
    }
}

void FlightController::writeHeader() {
    String header = Logging::generateCsvHeader();
    dataFile.println(header);
    dataFile.flush();
}

void FlightController::writeToRingBuffer() {
    if (!isRecording || !sdInitialized) return;

    static uint32_t rowCounter = 0;
    rowCounter++;

    // Get snapshots from all components
    const GnssSnapshot& gnssSnap = gnss.getLastSnapshot();
    const ImuSnapshot& imuSnap = imu.getLastSnapshot();

    // Compute control errors
    ControlErrors errors;
    errors.error_posN_m = positionSetpoint.posN - gnssData.posN;
    errors.error_posE_m = positionSetpoint.posE - gnssData.posE;
    errors.error_posD_m = positionSetpoint.posD - gnssData.posD;
    errors.error_pitch_deg = (posCtrlOutput.attitudeSetpoint.pitch - attitudeAngle.pitch) * RAD_TO_DEG;  // rad to deg
    errors.error_yaw_deg = (posCtrlOutput.attitudeSetpoint.yaw - attitudeAngle.yaw) * RAD_TO_DEG;        // rad to deg

    // Pack into CSV format using snprintf
    // Buffer must be large enough to hold the entire CSV line without truncation
    char line[2048];
    size_t len = Logging::packSnapshotCsv(line, sizeof(line),
                                          (uint32_t)micros(),
                                          flightTimeSeconds,
                                          attitude, imuAcc, gnssData,
                                          batteryStatus, actuators,
                                          gnssSnap, imuSnap,
                                          positionSetpoint, posCtrlOutput,
                                          posCtrl.N_integral_, posCtrl.E_integral_, posCtrl.D_integral_,
                                          errors, attitudeAngle);

    // Check for errors: packSnapshotCsv returns 0 on failure or if truncated
    if (len <= 0 || len >= sizeof(line) - 1) {
        // Line was truncated or failed to format
        droppedLines++;
        return;
    }

    // Append newline
    line[len] = '\n';
    len++;

    // Fast: write the preformatted bytes into the ring buffer
    size_t written = rb.write((const uint8_t*)line, len);

    // If ringbuffer overflowed or write didn't fit, rb.getWriteError() becomes true
    if (rb.getWriteError() || written != len) {
        // drop policy: drop this line and increment counter
        droppedLines++;
        // clear the write error so future writes succeed
        rb.clearWriteError();
    }
}

void FlightController::writeBufferToSD() {
    // Systematically drain ring buffer in 512-byte sectors until less than 512 bytes remain
    // CRITICAL FIX: Write multiple sectors at once if ring buffer is getting full
    // This prevents the ring buffer from filling up and blocking writes
    // Write aggressively when buffer usage is high (>50% = 10240 bytes)
    size_t n = rb.bytesUsed();

    static uint32_t sdWriteCallCount = 0;
    sdWriteCallCount++;
    if (sdWriteCallCount % 20 == 0) {  // Every 20 calls (~267ms at 250Hz IMU)
        // Serial.print("[SD_WRITE_CHECK] bytesUsed=");
        // Serial.print(n);
        // Serial.print(" isBusy=");
        // Serial.print(dataFile.isBusy());
        // Serial.print(" filePos=");
        // Serial.println(dataFile.curPosition());
    }

    if (n >= 10240 && !dataFile.isBusy()) {
        // Write up to 4 sectors (2048 bytes) at a time to catch up
        for (int i = 0; i < 4; i++) {
            size_t remaining = rb.bytesUsed();
            if (remaining < 512) break;  // Not enough for a full sector

            size_t bytesWrittenToSD = rb.writeOut(512);
            if (bytesWrittenToSD == 0) {
                isRecording = false;
                sdInitialized = false;
                return;
            }
        }
    } else if (n >= 512 && !dataFile.isBusy()) {
        // write out exactly 512 bytes from ring buffer into the file.
        // writeOut returns number bytes written, or 0 on failure.
        size_t bytesWrittenToSD = rb.writeOut(512);

        if (bytesWrittenToSD == 0) {
            isRecording = false;
            sdInitialized = false;
            return;
        }
    }
}