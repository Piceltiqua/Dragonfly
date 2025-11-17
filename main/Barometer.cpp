#include "Barometer.hpp"

void Barometer::setup() {
    bmp_.begin_SPI(BMP_CS);
    bmp_.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp_.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp_.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp_.setOutputDataRate(BMP3_ODR_50_HZ);
    bmp_.performReading();
}

void Barometer::read() {
    if (bmp_.performReading()) {
        if (!referenceSet) {
            setReference();
        }
        barometerData_.altBaro = referenceAltitude - bmp_.readAltitude(SEALEVELPRESSURE_HPA);
    }
}

void Barometer::setReference() {
    referenceAltitude = bmp_.readAltitude(SEALEVELPRESSURE_HPA);
    referenceSet = true;
}