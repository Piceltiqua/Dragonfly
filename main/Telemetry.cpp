#include "FC.hpp"

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
    memcpy(p, &gnssData.posN, sizeof(gnssData.posN));
    p += sizeof(gnssData.posN);
    memcpy(p, &gnssData.posE, sizeof(gnssData.posE));
    p += sizeof(gnssData.posE);
    memcpy(p, &gnssData.posD, sizeof(gnssData.posD));
    p += sizeof(gnssData.posD);
    memcpy(p, &gnssData.velN, sizeof(gnssData.velN));
    p += sizeof(gnssData.velN);
    memcpy(p, &gnssData.velE, sizeof(gnssData.velE));
    p += sizeof(gnssData.velE);
    memcpy(p, &gnssData.velD, sizeof(gnssData.velD));
    p += sizeof(gnssData.velD);

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

    memcpy(p, &actuators.motorThrust, sizeof(actuators.motorThrust));
    p += sizeof(actuators.motorThrust);

    *p++ = actuators.legsPosition;

    memcpy(p, &actuators.gimbalXAngle, sizeof(actuators.gimbalXAngle));
    p += sizeof(actuators.gimbalXAngle);
    memcpy(p, &actuators.gimbalYAngle, sizeof(actuators.gimbalYAngle));
    p += sizeof(actuators.gimbalYAngle);

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