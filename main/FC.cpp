#include "FC.hpp"

FlightController::FlightController()
    : imu(imuAcc, attitude),
      gnss(attitude, gnssData),
      command(actuators),
      battery(batteryStatus),
      attCtrl(attitude, attitudeSetpoint, actuators),
      posCtrl(gnssData, positionSetpoint, attitudeSetpoint)

{
    attitudeSetpoint.attitudeSetpoint.pitch = 0.0f;
    attitudeSetpoint.attitudeSetpoint.yaw = 0.0f;
    attitudeSetpoint.momentArm = moment_arm_legs_down;
    attitudeSetpoint.thrustCommand = 0.0f;

    // We target to hover at 1 meter above ground level
    positionSetpoint.posN =  0.0f;
    positionSetpoint.posE =  0.0f;
    positionSetpoint.posD = -1.0f;
    positionSetpoint.velN =  0.0f;
    positionSetpoint.velE =  0.0f;
    positionSetpoint.velD =  0.0f;
}

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
    // Emergency stop
    if (digitalRead(BUTTON_PIN) == HIGH) {
        Serial.println("Emergency stop");
        command.commandMotorsPercent(0, 0);
        while (true) {
            delay(1000);
        }
    }

    // Receive commands
    while (Serial1.available()) {
        int b = Serial1.read();
        if (b >= 0) processIncomingByte((uint8_t)b);
    }

    // Attitude loop
    if (IMUTimer >= IMU_PERIOD_US) {
        IMUTimer -= IMU_PERIOD_US;
        flightTimeSeconds = (float)millis() / 1000.0f;

        imu.read();
        smooth_imuread(attitude.wx, attitude.wy, attitude.wz);

        if (AttitudeControlled == true) {
            attCtrl.control();
            command.commandGimbal(actuators.servoXAngle,actuators.servoYAngle);
        }

        battery.readVoltage();
        battery.readCurrent();
        battery.integrateCurrentDraw();

        // Position loop
        if (gnss.read()) {
            // Faire une loop pour vérifier le fix et attérir si pas de fix pendant trop longtemps
            if (gnssData.fixType == 6) {
            }
            if (PositionControlled == true) {
                float dt = (micros() - lastGNSStime) / 1e6f;
                posCtrl.control(dt);
            }
            lastGNSStime = micros();
        }

        // Write to SD
        if (isRecording && SD.mediaPresent()) {
            writeToRingBuffer();
            writeBufferToSD();
        } else if (!SD.mediaPresent()) {
            isRecording = false;    // stop recording if SD removed
            sdInitialized = false;  // force re-init on next start
        }
    }

    // Telemetry sender loop
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
                    attitudeSetpoint.momentArm = moment_arm_legs_down;
                } else if (val == LEGS_RETRACTED) {
                    command.retractLegs();
                    attitudeSetpoint.momentArm = moment_arm_legs_up;
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
                if (ctrl == 0xC3) {
                    // enable attitude controller
                    AttitudeControlled = true;
                    attitudeSetpoint.attitudeSetpoint.pitch = 0.0f;
                    attitudeSetpoint.attitudeSetpoint.yaw = 0.0f;
                    // Calibrer le controlleur d'attitude ici
                    // led color orange
                    command.setLedColor(1, 0.5, 0);
                    Serial.println("Attitude controller enabled");

                } else if (ctrl == 0xCC) {
                    command.setLedColor(1, 0, 0);
                    AttitudeControlled = true;
                    PositionControlled = true;
                    posCtrl.init();
                    Serial.println("Position and attitude controller enabled");

                } else if (ctrl == 0xB7) {
                    AttitudeControlled = false;
                    PositionControlled = false;
                }
            }

            break;

        case MSG_ORIG:
            gnss.setReference(gnssData.lat, gnssData.lon, gnssData.alt);
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
                // ctrlOutput.thrust = (thrust1) * 9.81f / 1000.0f; // total thrust in N
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

void FlightController::smooth_imuread(float& wx, float& wy, float& wz) {
    // ring buffer implementation
    static std::vector<float> wx_buffer;
    static std::vector<float> wy_buffer;
    static std::vector<float> wz_buffer;

    const int buffer_size = 20;
    static int index = 0;
    if (wx_buffer.size() < buffer_size) {
        wx_buffer.push_back(wx);
        wy_buffer.push_back(wy);
        wz_buffer.push_back(wz);
    } else {
        wx_buffer[index] = wx;
        wy_buffer[index] = wy;
        wz_buffer[index] = wz;
        index = (index + 1) % buffer_size;
    }
    // claculate the average
    float wx_sum = 0.0f;
    float wy_sum = 0.0f;
    float wz_sum = 0.0f;
    for (int i = 0; i < wx_buffer.size(); i++) {
        wx_sum += wx_buffer[i];
        wy_sum += wy_buffer[i];
        wz_sum += wz_buffer[i];
    }
    wx = wx_sum / wx_buffer.size();
    wy = wy_sum / wy_buffer.size();
    wz = wz_sum / wz_buffer.size();
}

