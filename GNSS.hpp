#pragma once
#include "utils.hpp"
#include "ubx.h"

class GNSS {
public:
    GNSS(GNSSData &gnssData) : gnssData_(gnssData) {}

    void setup();
    void read();

private:
    const double a = 6378137.0;
    const double f = 1 / 298.257223563;
    const double e2 = f * (2 - f);
    double N0 = 0.0;
    double M0 = 0.0;

    bool initialFix_ = false;
    bfs::Ubx ubx_{&Serial5};
    GNSSData& gnssData_;
}