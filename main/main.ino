#include "FC.hpp"

FlightController FC;

void setup() {
    FC.setup();
}

void loop() {
    FC.readSensors();
}