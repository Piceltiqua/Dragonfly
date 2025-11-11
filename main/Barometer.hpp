#ifndef BAROMETER_H
#define BAROMETER_H

#include "Utils.hpp"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

class Barometer {
public:
    Barometer(BarometerData &baroData) : baroData_(baroData) {}

    void setup();
    void read();
    void setReference(float pressure)

private:
    Adafruit_BMP3XX bmp_;
    bool ReferenceSet = false;
    float ReferencePressure;

    BarometerData &baroData_;
};

#endif