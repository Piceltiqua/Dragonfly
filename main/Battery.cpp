#include "Battery.hpp"

void Battery::setup() {
    pinMode(BATTERY_VOLTAGE_PIN, INPUT);
    pinMode(BATTERY_CURRENT_PIN, INPUT);
}

void Battery::readVoltage() {
    batteryVoltage = (analogRead(BATTERY_VOLTAGE_PIN) / 1023.0f) * 33.0f;
    batteryStatus_.batteryVoltage = static_cast<uint16_t>(batteryVoltage * 1000.0f);  // send in mV
}

void Battery::readCurrent() {
    currentDraw = (analogRead(BATTERY_CURRENT_PIN) / 1023.0f) * 50.0f;
    batteryStatus_.currentDraw = static_cast<uint16_t>(currentDraw * 1000.0f);  // send in mA
}

void Battery::integrateCurrentDraw() {
    float dt = (micros() - lastIntegrationTime) / 1000000.0f;  // seconds
    lastIntegrationTime = micros();
    currentConsumed += currentDraw * dt / 3600.0f * 1000.0f;      // mAh
    levelOfCharge_mAh -= (currentDraw * dt) / 3600.0f * 1000.0f;  // mAh
    levelOfCharge_percent = (levelOfCharge_mAh / BATTERY_TOTAL_CAPACITY_MAH) * 100.0f;
    batteryStatus_.batteryLevel = static_cast<uint8_t>(levelOfCharge_percent);
    batteryStatus_.currentConsumed = static_cast<int16_t>(currentConsumed);
}

void Battery::setCharge(uint16_t charge) {
    levelOfCharge_mAh = charge;
}