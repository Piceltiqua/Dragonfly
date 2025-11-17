#ifndef BAROMETER_H
#define BAROMETER_H

#include "Utils.hpp"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA 1013.25
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

class Barometer {
public:
    Barometer(BarometerData &barometerData) : barometerData_(barometerData) {}

    void setup();
    void read();
    void setReference();

private:
    Adafruit_BMP3XX bmp_;
    bool referenceSet = false;
    float referenceAltitude;

    BarometerData &barometerData_;
};

#endif