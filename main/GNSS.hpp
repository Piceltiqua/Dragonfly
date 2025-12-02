// GNSS.hpp
#ifndef GNSS_H
#define GNSS_H

#include "Utils.hpp"
#include "ubx.h"

extern bfs::Ubx ubx;

class GNSS {
public:
    GNSS(Attitude& attitude,
         GNSSData& gnssData)
        : attitude_(attitude),
          gnssData_(gnssData) {}

    void setup();
    bool read();
    String getDate();
    void setReference(double lat0, double lon0, double alt0);

private:
    static constexpr double a = 6378137.0;
    static constexpr double f = 1.0 / 298.257223563;
    static constexpr double e2 = f * (2.0 - f);
    static constexpr uint32_t GNSS_VEL_LATENCY_US = 130000;
    static constexpr float alpha = 0.2f;  // velocity low-pass filter coefficient

    void computeCGPositionNED();
    void computeCGVelocityNED();
    bool interpolateIMUSample();

    double N0 = 0.0;
    double M0 = 0.0;

    bool newReading = false;
    bool reference_ = false;
    Attitude& attitude_;
    GNSSData& gnssData_;

    Eigen::Vector3f omega_ned_interp;
    Eigen::Vector3f r_ant_ned_interp;
    Eigen::Vector3f v_ant_ned;

    uint32_t t_GNSS_read_us = 0;
};

#endif
