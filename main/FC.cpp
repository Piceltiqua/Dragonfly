#include "FC.hpp"

FlightController::FlightController()
    : imu(imuAcc, attitude),
      gnss(attitude, gnssData),
      battery(batteryStatus)
// attitudeCtrl(attitude, actuatorCmds),
// positionCtrl(posvel, targetAttitude, waypoints)
{}

void FlightController::setup() {
    imu.setup(IMU_FREQ_HZ);
    delay(100);
    gnss.setup();
    delay(100);
    command.setup();
    delay(100);
    battery.setup();
    delay(100);
    initializeSD();
    delay(100);

    // Serial.begin(115200);
    Serial1.begin(57600);  // THIS MUST NOT BE BEFORE THE OTHER SETUPS
    Serial1.addMemoryForWrite(extra_tx_mem, sizeof(extra_tx_mem));
    delay(1000);
}

void FlightController::readSensors() {
    while (Serial1.available()) {
        int b = Serial1.read();
        if (b >= 0) processIncomingByte((uint8_t)b);
    }

    // Attitude loop
    if (IMUTimer >= IMU_PERIOD_US) {
        // Receive commands
        static uint32_t sensorReadCounter = 0;
        sensorReadCounter++;

        IMUTimer -= IMU_PERIOD_US;
        flightTimeSeconds = (float)millis() / 1000.0f;

        // uint32_t t0_imuread = micros();
        imu.read();
        // Serial.print("IMU read time (us): ");
        // Serial.println(micros() - t0_imuread);

        battery.readVoltage();
        battery.readCurrent();
        battery.integrateCurrentDraw();

        if (gnss.read()) {
            if (gnssData.fixType == 6) {
                posvel.posN = gnssData.posN;
                posvel.posE = gnssData.posE;
                posvel.posD = gnssData.posD;
                posvel.velN = gnssData.velN;
                posvel.velE = gnssData.velE;
                posvel.velD = gnssData.velD;
            }
        }

        if (isRecording && SD.mediaPresent()) {
            writeToRingBuffer();
            writeBufferToSD();
        } else if (!SD.mediaPresent()) {
            isRecording = false;    // stop recording if SD removed
            sdInitialized = false;  // force re-init on next start
        }
    }

    // Telemetry loop
    if (telemTimer >= TELEMETRY_PERIOD_US) {
        telemTimer -= TELEMETRY_PERIOD_US;

        sendTelemetry();
    }
}

void FlightController::executeCommandFromPayload(const uint8_t* payload, size_t payloadLen) {
    if (payloadLen < 1) return;

    // payload[0] = message_type
    uint8_t header = payload[0];

    // commands mapping:
    switch (header) {
        case MSG_FLY:
            // start flight
            // implement your flight start logic here
            // NOTE: we already echoed the frame back before executing
            command.setLedColor(0, 0, 0);
            break;

        case MSG_LEG: {
            if (payloadLen >= 2) {
                uint8_t val = payload[1];
                // val will be 0x9D or 0xA9 per GUI
                actuators.legsPosition = val;  // example: set state to deployed/
                command.setLedColor(0, 0, 1);
                if (val == LEGS_DEPLOYED) {
                    command.extendLegs();
                } else if (val == LEGS_RETRACTED) {
                    command.retractLegs();
                }
            }

            break;
        }

        case MSG_BAT: {
            if (payloadLen >= 3) {
                uint16_t charge_mah = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
                battery.setCharge(charge_mah);
                command.setLedColor(0, 1, 0);
            }

            break;
        }

        case MSG_CTRL:
            if (payloadLen >= 2) {
                uint8_t ctrl = payload[1];
                command.setLedColor(1, 0, 0);
                // handle enabling/disabling controllers
            }

            break;

        case MSG_ORIG:
            gnss.setReference(gnssData.lat, gnssData.lon, gnssData.alt);
            posvel.posN = 0.0f;
            posvel.posE = 0.0f;
            posvel.posD = 0.0f;
            posvel.velN = 0.0f;
            posvel.velE = 0.0f;
            posvel.velD = 0.0f;
            command.setLedColor(1, 0, 1);
            tSetOrigin = micros();
            break;

        case MSG_ENG:
            if (payloadLen >= 3) {
                uint8_t m1 = payload[1];  // throttle (%)
                uint8_t m2 = payload[2];
                // command.commandMotorsPercent(m1, m2);
                float thrust1 = map(m1, 0, 100, 0, 2060);
                float thrust2 = map(m2, 0, 100, 0, 2060);
                command.commandMotorsThrust(thrust1, thrust2);
                command.setLedColor(1, 1, 0);
            }
            break;

        case MSG_STOP:
            command.commandMotorsPercent(0, 0);  // stop motors immediately
            command.setLedColor(1, 1, 1);
            break;

        case MSG_LAND:
            // landing logic
            break;

        case MSG_REC:
            if (payloadLen >= 2) {
                uint8_t rec = payload[1];
                handleRecordingMessage(rec);
            }
            break;

        case MSG_GIMBAL:
            if (payloadLen >= 3) {
                int8_t gimbalX = payload[1];  // gimbal angle in degrees (-6/+6)
                int8_t gimbalY = payload[2];
                command.commandGimbal(gimbalX, gimbalY);
                command.setLedColor(1, 1, 0);
            }
            break;

        default:
            Serial.print("Unknown command type ");
            Serial.println(header, HEX);
            break;
    }
}

void FlightController::printState() {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(posvel.posN);
    Serial.print(",");
    Serial.print(posvel.posE);
    Serial.print(",");
    Serial.print(posvel.posD);
    Serial.print(",");
    Serial.print(posvel.velN);
    Serial.print(",");
    Serial.print(posvel.velE);
    Serial.print(",");
    Serial.print(posvel.velD);
    Serial.print(",");

    Serial.print(attitude.qw);
    Serial.print(",");
    Serial.print(attitude.qi);
    Serial.print(",");
    Serial.print(attitude.qj);
    Serial.print(",");
    Serial.print(attitude.qk);
    Serial.print(",");
    Serial.print(attitude.wx);
    Serial.print(",");
    Serial.print(attitude.wy);
    Serial.print(",");
    Serial.print(attitude.wz);
    Serial.print(",");
    Serial.print(imuAcc.ax_NED);
    Serial.print(",");
    Serial.print(imuAcc.ay_NED);
    Serial.print(",");
    Serial.print(imuAcc.az_NED);
    Serial.print(",");
    Serial.print(gnssData.lat);
    Serial.print(",");
    Serial.print(gnssData.lon);
    Serial.print(",");
    Serial.print(gnssData.alt);
    Serial.print(",");
    Serial.print(gnssData.posN);
    Serial.print(",");
    Serial.print(gnssData.posE);
    Serial.print(",");
    Serial.print(gnssData.posD);
    Serial.print(",");
    Serial.print(gnssData.velN);
    Serial.print(",");
    Serial.print(gnssData.velE);
    Serial.print(",");
    Serial.print(gnssData.velD);
    Serial.print(",");
    Serial.print(gnssData.horAcc);
    Serial.print(",");
    Serial.print(gnssData.vertAcc);
    Serial.print(",");
    Serial.print(gnssData.numSV);
    Serial.print(",");
    Serial.println(gnssData.fixType);
}

uint16_t FlightController::crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void FlightController::writeEscapedByte(uint8_t b) {
    if (b == STX || b == DLE) {
        Serial1.write(DLE);
        Serial1.write(b ^ XOR_MASK);
    } else {
        Serial1.write(b);
    }
}

void FlightController::sendRawFramed(const uint8_t* unescaped, size_t unescapedLen) {
    Serial1.write(STX);
    for (size_t i = 0; i < unescapedLen; ++i) writeEscapedByte(unescaped[i]);
    Serial1.write(STX);
    // Serial1.flush();
}

void FlightController::buildPackedPayload(uint8_t* buf, size_t& outLen) {
    uint8_t* p = buf;
    // message type
    *p++ = MSG_DATA;

    // append floats/doubles etc. use memcpy to preserve bit patterns
    memcpy(p, &posvel.posN, sizeof(posvel.posN));
    p += sizeof(posvel.posN);
    memcpy(p, &posvel.posE, sizeof(posvel.posE));
    p += sizeof(posvel.posE);
    memcpy(p, &posvel.posD, sizeof(posvel.posD));
    p += sizeof(posvel.posD);
    memcpy(p, &posvel.velN, sizeof(posvel.velN));
    p += sizeof(posvel.velN);
    memcpy(p, &posvel.velE, sizeof(posvel.velE));
    p += sizeof(posvel.velE);
    memcpy(p, &posvel.velD, sizeof(posvel.velD));
    p += sizeof(posvel.velD);

    memcpy(p, &imuAcc.ax_NED, sizeof(imuAcc.ax_NED));
    p += sizeof(imuAcc.ax_NED);
    memcpy(p, &imuAcc.ay_NED, sizeof(imuAcc.ay_NED));
    p += sizeof(imuAcc.ay_NED);
    memcpy(p, &imuAcc.az_NED, sizeof(imuAcc.az_NED));
    p += sizeof(imuAcc.az_NED);
    memcpy(p, &imuAcc.accuracy_status, sizeof(imuAcc.accuracy_status));
    p += sizeof(imuAcc.accuracy_status);

    memcpy(p, &attitude.wx, sizeof(attitude.wx));
    p += sizeof(attitude.wx);
    memcpy(p, &attitude.wy, sizeof(attitude.wy));
    p += sizeof(attitude.wy);
    memcpy(p, &attitude.wz, sizeof(attitude.wz));
    p += sizeof(attitude.wz);

    memcpy(p, &attitude.qw, sizeof(attitude.qw));
    p += sizeof(attitude.qw);
    memcpy(p, &attitude.qi, sizeof(attitude.qi));
    p += sizeof(attitude.qi);
    memcpy(p, &attitude.qj, sizeof(attitude.qj));
    p += sizeof(attitude.qj);
    memcpy(p, &attitude.qk, sizeof(attitude.qk));
    p += sizeof(attitude.qk);

    memcpy(p, &gnssData.lat, sizeof(gnssData.lat));
    p += sizeof(gnssData.lat);
    memcpy(p, &gnssData.lon, sizeof(gnssData.lon));
    p += sizeof(gnssData.lon);
    memcpy(p, &gnssData.alt, sizeof(gnssData.alt));
    p += sizeof(gnssData.alt);

    memcpy(p, &gnssData.horAcc, sizeof(gnssData.horAcc));
    p += sizeof(gnssData.horAcc);
    memcpy(p, &gnssData.vertAcc, sizeof(gnssData.vertAcc));
    p += sizeof(gnssData.vertAcc);

    *p++ = gnssData.numSV;
    *p++ = gnssData.fixType;

    memcpy(p, &batteryStatus.currentDraw, sizeof(batteryStatus.currentDraw));
    p += sizeof(batteryStatus.currentDraw);
    memcpy(p, &batteryStatus.currentConsumed, sizeof(batteryStatus.currentConsumed));
    p += sizeof(batteryStatus.currentConsumed);
    memcpy(p, &batteryStatus.batteryVoltage, sizeof(batteryStatus.batteryVoltage));
    p += sizeof(batteryStatus.batteryVoltage);
    *p++ = batteryStatus.batteryLevel;

    uint32_t UPTIME = millis();
    memcpy(p, &UPTIME, sizeof(UPTIME));
    p += sizeof(UPTIME);

    memcpy(p, &actuators.motor1Throttle, sizeof(actuators.motor1Throttle));
    p += sizeof(actuators.motor1Throttle);
    memcpy(p, &actuators.motor2Throttle, sizeof(actuators.motor2Throttle));
    p += sizeof(actuators.motor2Throttle);

    *p++ = actuators.legsPosition;

    memcpy(p, &actuators.servoXAngle, sizeof(actuators.servoXAngle));
    p += sizeof(actuators.servoXAngle);
    memcpy(p, &actuators.servoYAngle, sizeof(actuators.servoYAngle));
    p += sizeof(actuators.servoYAngle);

    memcpy(p, &isRecording, sizeof(isRecording));
    p += sizeof(isRecording);

    memcpy(p, &flightTimeSeconds, sizeof(flightTimeSeconds));
    p += sizeof(flightTimeSeconds);

    outLen = p - buf;
}

void FlightController::processCompleteUnescapedFrame(const uint8_t* buf, size_t len) {
    // buf[0..1] = len low, len high
    if (len < 2) return;
    uint16_t payloadLen = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    if ((size_t)(payloadLen + 2) != len) {
        // mismatch (this shouldn't happen if caller provided correct unescaped len)
        return;
    }
    const uint8_t* payloadWithCrc = buf + 2;
    size_t payloadWithCrcLen = payloadLen;
    if (payloadWithCrcLen < 3) return;  // needs at least type + crc

    // verify CRC
    uint16_t receivedCrc = (uint16_t)payloadWithCrc[payloadWithCrcLen - 2] | ((uint16_t)payloadWithCrc[payloadWithCrcLen - 1] << 8);
    uint16_t calc = crc16_ccitt(payloadWithCrc, payloadWithCrcLen - 2);
    if (receivedCrc != calc) {
        Serial.println("Incoming CRC mismatch, dropping frame");
        return;
    }

    // if this is a command (not telemetry), echo it back immediately and then execute
    uint8_t msgType = payloadWithCrc[0];
    if (msgType != MSG_DATA) {
        // echo the entire payloadWithCrc framed (len + payload) back as ack
        uint8_t framed[2 + 1024];
        framed[0] = (uint8_t)(payloadWithCrcLen & 0xFF);
        framed[1] = (uint8_t)((payloadWithCrcLen >> 8) & 0xFF);
        memcpy(framed + 2, payloadWithCrc, payloadWithCrcLen);
        sendRawFramed(framed, 2 + payloadWithCrcLen);

        // execute command (Teensy executes every received command; REC handled idempotently)
        executeCommandFromPayload(payloadWithCrc, payloadWithCrcLen);
        return;
    }
}

void FlightController::processIncomingByte(uint8_t b) {
    if (!inFrame) {
        if (b == STX) {
            inFrame = true;
            escapeNext = false;
            frameBufLen = 0;
        }
        return;
    }
    if (escapeNext) {
        uint8_t orig = b ^ XOR_MASK;
        if (frameBufLen < MAX_FRAME_BUFFER) frameBuf[frameBufLen++] = orig;
        escapeNext = false;
    } else if (b == DLE) {
        escapeNext = true;
    } else if (b == STX) {
        // end-of-frame (STX) encountered -> process frameBuf which contains unescaped bytes
        // we pass to processCompleteUnescapedFrame if it looks valid
        if (frameBufLen >= 2) {
            uint16_t payloadLen = (uint16_t)frameBuf[0] | ((uint16_t)frameBuf[1] << 8);
            if ((size_t)(payloadLen + 2) == frameBufLen) {
                processCompleteUnescapedFrame(frameBuf, frameBufLen);
            } else {
                // partial or inconsistent frame; drop and resync
            }
        }
        // reset and allow next frame
        inFrame = false;
        escapeNext = false;
        frameBufLen = 0;
    } else {
        if (frameBufLen < MAX_FRAME_BUFFER) frameBuf[frameBufLen++] = b;
    }
}

// build the escaped frame into outBuf and return outLen
size_t buildEscapedFramed(const uint8_t* unescaped, size_t unescapedLen, uint8_t* outBuf, size_t outBufMax) {
    size_t outPos = 0;
    if (outPos < outBufMax) outBuf[outPos++] = STX;
    for (size_t i = 0; i < unescapedLen; ++i) {
        uint8_t b = unescaped[i];
        if (b == STX || b == DLE) {
            if (outPos + 2 > outBufMax) return 0;  // overflow
            outBuf[outPos++] = DLE;
            outBuf[outPos++] = b ^ XOR_MASK;
        } else {
            if (outPos + 1 > outBufMax) return 0;  // overflow
            outBuf[outPos++] = b;
        }
    }
    if (outPos < outBufMax) outBuf[outPos++] = STX;
    return outPos;
}

// Non-blocking send: writes only if TX buffer has enough room
bool FlightController::trySendPayloadWithCrc(const uint8_t* payloadWithCrc, size_t payloadLen) {
    // framed unescaped = [len_low, len_hi, payloadWithCrc...]
    static uint8_t framedUnescaped[2 + 1024];
    framedUnescaped[0] = payloadLen & 0xFF;
    framedUnescaped[1] = (payloadLen >> 8) & 0xFF;
    memcpy(framedUnescaped + 2, payloadWithCrc, payloadLen);
    size_t unescapedLen = 2 + payloadLen;

    // build escaped frame into a local buffer
    static uint8_t outBuf[2 + 2 * (1024 + 2)];  // worst-case space
    size_t outLen = buildEscapedFramed(framedUnescaped, unescapedLen, outBuf, sizeof(outBuf));
    if (outLen == 0) return false;  // trouble building

    // Safe to write without blocking since we have a big TX buffer
    Serial1.write(outBuf, outLen);
    return true;
}

void FlightController::sendTelemetry() {
    // Build packed payload (message_type + fields) into a local buffer
    uint8_t rawBuf[1024];
    size_t payloadNoCrcLen = 0;

    // reuse buildPackedPayload idea
    buildPackedPayload(rawBuf + 0, payloadNoCrcLen);  // returns payloadNoCrc in rawBuf

    // compute CRC over payloadNoCrc
    uint16_t crc = crc16_ccitt(rawBuf, payloadNoCrcLen);

    // build payloadWithCrc in separate buffer
    uint8_t payloadWithCrc[1024];
    memcpy(payloadWithCrc, rawBuf, payloadNoCrcLen);
    payloadWithCrc[payloadNoCrcLen++] = crc & 0xFF;
    payloadWithCrc[payloadNoCrcLen++] = (crc >> 8) & 0xFF;

    // send framed
    trySendPayloadWithCrc(payloadWithCrc, payloadNoCrcLen);
}

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
            if (sdInitialized && gnssData.fixType >= 5) {
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

    // Pack into CSV format using snprintf
    // Buffer must be large enough to hold the entire CSV line without truncation
    char line[2048];  // Reduced size since EKF data is removed
    size_t len = Logging::packSnapshotCsv(line, sizeof(line),
                                          (uint32_t)micros(),
                                          posvel, attitude, imuAcc, gnssData,
                                          batteryStatus, actuators,
                                          gnssSnap, imuSnap);

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
        // Debug: aggressive drain
        // Serial.print("[RB_AGGRESSIVE_DRAIN] Starting drain at bytesUsed=");
        // Serial.println(n);

        // Write up to 4 sectors (2048 bytes) at a time to catch up
        for (int i = 0; i < 4; i++) {
            size_t remaining = rb.bytesUsed();
            if (remaining < 512) break;  // Not enough for a full sector

            size_t bytesWrittenToSD = rb.writeOut(512);
            if (bytesWrittenToSD == 0) {
                // Serial.print("[SD_FATAL_ERROR] At file position: ");
                // Serial.println(dataFile.curPosition());
                // handle fatal: stop recording
                isRecording = false;
                sdInitialized = false;
                return;
            }

            if (i == 3) {
                // Serial.print("[RB_AGGRESSIVE_DRAIN] Wrote 4 sectors, bytesUsed now=");
                // Serial.println(rb.bytesUsed());
            }
        }
    } else if (n >= 512 && !dataFile.isBusy()) {
        // Debug: Log before writeOut
        if (sdWriteCallCount % 20 == 0) {
            // Serial.print("[RB_WRITEOUT_START] n=");
            // Serial.print(n);
            // Serial.print(" writeSize=512 filePos=");
            // Serial.println(dataFile.curPosition());
        }

        // write out exactly 512 bytes from ring buffer into the file.
        // writeOut returns number bytes written, or 0 on failure.
        size_t bytesWrittenToSD = rb.writeOut(512);

        // Debug: Log after writeOut
        if (sdWriteCallCount % 20 == 0) {
            // Serial.print("[RB_WRITEOUT_END] bytesWritten=");
            // Serial.print(bytesWrittenToSD);
            // Serial.print(" newBytesUsed=");
            // Serial.print(rb.bytesUsed());
            // Serial.print(" filePos=");
            // Serial.println(dataFile.curPosition());
        }

        if (bytesWrittenToSD == 0) {
            // Serial.print("[SD_FATAL_ERROR] At file position: ");
            // Serial.println(dataFile.curPosition());
            // handle fatal: stop recording
            isRecording = false;
            sdInitialized = false;
            return;
        }
    }
}
