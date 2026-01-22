#include "FC.hpp"

FlightController::FlightController()
    : imu(imuAcc, attitude),
      gnss(attitude, gnssData),
      command(actuators),
      battery(batteryStatus),
      attCtrl(attitude, attitudeAngle, posCtrlOutput, actuators),
      posCtrl(gnssData, attitudeAngle, positionSetpoint, posCtrlOutput),
      rollCtrl(actuators),
      waypointManager(positionSetpoint)

{
    posCtrlOutput.attitudeSetpoint.pitch = 0.0f;
    posCtrlOutput.attitudeSetpoint.yaw = 0.0f;
    posCtrlOutput.momentArm = moment_arm_legs_down;
    posCtrlOutput.thrustCommand = 0.0f;

    positionSetpoint.posN = 0.0f;
    positionSetpoint.posE = 0.0f;
    positionSetpoint.posD = 0.0f;
    positionSetpoint.velN = 0.0f;
    positionSetpoint.velE = 0.0f;
    positionSetpoint.velD = 0.0f;
    positionSetpoint.accffN = 0.0f;
    positionSetpoint.accffE = 0.0f;
    positionSetpoint.accffD = 0.0f;
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

    Serial1.begin(57600);  // THIS MUST NOT BE BEFORE THE OTHER SETUPS
    Serial1.addMemoryForWrite(extra_tx_mem, sizeof(extra_tx_mem));
    delay(1000);
}

void FlightController::loop() {
    // Emergency stop
    if (digitalRead(BUTTON_PIN) == HIGH) {
        Serial.println("Emergency stop");
        command.commandMotorsThrust(0, 0);
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

        imu.read();
        smooth_imuread(attitude.wx, attitude.wy, attitude.wz);

        if (AttitudeControlled) {
            attCtrl.control();
            command.commandGimbal(actuators.gimbalXAngle, actuators.gimbalYAngle);
        }
        command.commandMotorsThrust(actuators.motorThrust, deltaTimingRoll, batteryStatus.batteryVoltage);

        battery.readVoltage();
        battery.readCurrent();
        battery.integrateCurrentDraw();

        if (InFlight) {
            flightTimeSeconds = (static_cast<float>(millis()) - flightStartTime) / 1000.0f;
            if (!waypointManager.flying(flightTimeSeconds)) {
                InFlight = false;
                PositionControlled = false;
                AttitudeControlled = false;
                RollControlled = false;

                if (actuators.legsPosition == LEGS_DEPLOYED) {
                    deltaTimingRoll = rollCtrl.MOTOR_DEPLOYED_OFFSET;
                } else {
                    deltaTimingRoll = rollCtrl.MOTOR_RETRACTED_OFFSET;
                }
                posCtrlOutput.thrustCommand = 0.0f;
                actuators.motorThrust = 0;
                command.commandMotorsThrust(0, 0);  // stop motors immediately
                Serial.println("Flight complete.");
            }
        }

        // Position loop
        if (gnss.read()) {
            // Faire une loop pour vérifier le fix et atterrir si pas de fix pendant trop longtemps
            if (gnssData.fixType == 6) {
            }
            
            if (PositionControlled) {
                float dt = (micros() - lastGNSStime) / 1e6f;
                posCtrl.control(dt);
            }
            lastGNSStime = micros();
            
            if (RollControlled) {
                deltaTimingRoll = rollCtrl.computeRollTimingOffsets(attitude.wz, actuators.legsPosition);
            }
            
            updateLedColorForRTKFix();
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
            if (gnssData.fixType < 6) {
                Serial.println("Cannot start flight: GNSS fix not sufficient.");
                InFlight = false;
                PositionControlled = false;
                AttitudeControlled = false;
                RollControlled = false;
                if (actuators.legsPosition == LEGS_DEPLOYED) {
                    deltaTimingRoll = rollCtrl.MOTOR_DEPLOYED_OFFSET;
                } else {
                    deltaTimingRoll = rollCtrl.MOTOR_RETRACTED_OFFSET;
                }
                return;
            }
            if (!waypointManager.init()) {
                Serial.println("Waypoint initialization failed. Aborting flight start.");
                InFlight = false;
                PositionControlled = false;
                AttitudeControlled = false;
                RollControlled = false;
                if (actuators.legsPosition == LEGS_DEPLOYED) {
                    deltaTimingRoll = rollCtrl.MOTOR_DEPLOYED_OFFSET;
                } else {
                    deltaTimingRoll = rollCtrl.MOTOR_RETRACTED_OFFSET;
                }
                return;
            }
            flightStartTime = static_cast<float>(millis());
            InFlight = true;
            gnss.setReference(gnssData.lat, gnssData.lon, gnssData.alt);
            PositionControlled = true;
            posCtrl.init();
            AttitudeControlled = true;
            RollControlled = true;

            break;

        case MSG_LEG: {
            if (payloadLen >= 2) {
                uint8_t val = payload[1];
                // val will be 0x9D or 0xA9 per GUI
                if (val == LEGS_DEPLOYED) {
                    command.extendLegs();
                    posCtrlOutput.momentArm = moment_arm_legs_down;
                } else if (val == LEGS_RETRACTED) {
                    command.retractLegs();
                    posCtrlOutput.momentArm = moment_arm_legs_up;
                }
            }
            break;
        }

        case MSG_BAT: {
            if (payloadLen >= 3) {
                uint16_t charge_mah = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
                battery.setCharge(charge_mah);
            }
            break;
        }

        case MSG_CTRL:
            if (payloadLen >= 2) {
                uint8_t ctrl = payload[1];
                if (ctrl == 0xC3) {
                    // Enable attitude controller
                    RollControlled     = true;
                    AttitudeControlled = true;
                    posCtrlOutput.attitudeSetpoint.pitch = 0.0f;
                    posCtrlOutput.attitudeSetpoint.yaw = 0.0f;
                    Serial.println("Attitude controller enabled");

                } else if (ctrl == 0xCC) {
                    // Enable position controller
                    RollControlled     = true;
                    AttitudeControlled = true;
                    PositionControlled = true;
                    gnss.setReference(gnssData.lat, gnssData.lon, gnssData.alt);
                    posCtrl.init();
                    Serial.println("Position and attitude controller enabled");

                } else if (ctrl == 0xB7) {
                    // Disable all controllers
                    RollControlled     = false;
                    AttitudeControlled = false;
                    PositionControlled = false;
                    if (actuators.legsPosition == LEGS_DEPLOYED) {
                        deltaTimingRoll = rollCtrl.MOTOR_DEPLOYED_OFFSET;
                    } else {
                        deltaTimingRoll = rollCtrl.MOTOR_RETRACTED_OFFSET;
                    }
                }
            }
            break;

        case MSG_ORIG:
            gnss.setReference(gnssData.lat, gnssData.lon, gnssData.alt);
            break;

        case MSG_ENG:
            if (payloadLen >= 3) {
                uint8_t m = payload[1];  // throttle (%)
                posCtrlOutput.thrustCommand = static_cast<int16_t>(-1.23e-3 * pow(m, 3) + 3.44e-1 * pow(m, 2) + 1.59 * m - 2.9);  // thrust in grams
                actuators.motorThrust = posCtrlOutput.thrustCommand;
            }
            break;

        case MSG_STOP:
            InFlight = false;
            PositionControlled = false;
            posCtrlOutput.thrustCommand = 0.0f;
            actuators.motorThrust = 0;
            command.commandMotorsThrust(0, 0);  // stop motors immediately
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
                int8_t gimbalX = payload[1];  // gimbal angle (+/- 5°)
                int8_t gimbalY = payload[2];
                actuators.gimbalXAngle = static_cast<float>(gimbalX);
                actuators.gimbalYAngle = static_cast<float>(gimbalY);
                command.commandGimbal(gimbalX, gimbalY);
            }
            break;

        default:
            Serial.print("Unknown command type ");
            Serial.println(header, HEX);
            break;
    }
}

void FlightController::updateLedColorForRTKFix() {
    // LED color lookup table for different RTK fix types
    if (gnssData.fixType == 6) {
        // RTK Fix, green
        command.setLedColor(0, 1, 0);
    } else if (gnssData.fixType == 5) {
        // RTK Float, yellow
        command.setLedColor(1, 1, 0);
    } else {
        // No fix, red
        command.setLedColor(1, 0, 0);
    }
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
