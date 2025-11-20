#ifndef BATTERY_H
#define BATTERY_H

#include "Utils.hpp"

static constexpr int BATTERY_VOLTAGE_PIN = A14;
static constexpr int BATTERY_CURRENT_PIN = A15;
static constexpr int BATTERY_TOTAL_CAPACITY_MAH = 1800;  // mAh

class Battery {
public:
    Battery(BatteryStatus& batStatus)
        : batteryStatus_(batStatus) {}
    void setup();
    void readVoltage();
    void readCurrent();
    void integrateCurrentDraw();
    void setCharge(uint16_t charge);

private:
    BatteryStatus& batteryStatus_;
    float levelOfCharge_percent = 100.0f;
    float levelOfCharge_mAh = 0.0f;
    float batteryVoltage = 0.0f;
    float currentDraw = 0.0f;
    float currentConsumed = 0.0f;
    unsigned long lastIntegrationTime = 0;
};

#endif