#include "Barometer.hpp"

Barometer::setup() {
    bmp_.begin_SPI(BMP_CS);
    bmp_.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp_.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp_.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp_.setOutputDataRate(BMP3_ODR_50_HZ);
}

Barometer::read() {
    if (bmp_.performReading()) {
        if (!ReferenceSet) {
            setReference(bmp_.pressure);
        }
        baroData_.altitude = bmp_.readAltitude(ReferencePressure);
    }
}

Barometer::setReference(float pressure) {
    ReferencePressure = pressure;
    ReferenceSet = true;
}