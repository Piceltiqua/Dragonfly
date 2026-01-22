// GNSS.hpp
#ifndef GNSS_H
#define GNSS_H

#include "Utils.hpp"
#include "ubx.h"

extern bfs::Ubx ubx;

struct GnssSnapshot {
    // From computeCGPositionNED()
    float r_ant_ned_x, r_ant_ned_y, r_ant_ned_z;  // Antenna offset in NED frame
    float p_ant_ned_x, p_ant_ned_y, p_ant_ned_z;  // Measured antenna position in NED
    float p_cg_ned_x, p_cg_ned_y, p_cg_ned_z;     // CG position in NED

    // From computeCGVelocityNED()
    float v_cg_ned_x, v_cg_ned_y, v_cg_ned_z;                          // CG velocity in NED
    float v_ant_ned_x, v_ant_ned_y, v_ant_ned_z;                       // Antenna velocity in NED
    float omega_ned_interp_x, omega_ned_interp_y, omega_ned_interp_z;  // Interpolated angular rates
    float r_ant_ned_interp_x, r_ant_ned_interp_y, r_ant_ned_interp_z;  // Interpolated antenna position
};

class GNSS {
public:
    GNSS(Attitude& attitude,
         GNSSData& gnssData)
        : attitude_(attitude),
          gnssData_(gnssData),
          last_snapshot_() {}

    void setup();
    bool read();
    String getDateFilename();
    void setReference(double lat0, double lon0, double alt0);
    const GnssSnapshot& getLastSnapshot() const { return last_snapshot_; }
    uint32_t getMeasurementTime() const { return t_GNSS_read_us; }

private:
    static constexpr double a = 6378137.0;
    static constexpr double f = 1.0 / 298.257223563;
    static constexpr double e2 = f * (2.0 - f);
    static constexpr uint32_t GNSS_VEL_LATENCY_US = 130000;
    static constexpr float alpha = 1.0f;  // velocity low-pass filter coefficient

    void computeCGPositionNED();
    void computeCGVelocityNED();
    bool interpolateIMUSample();

    double N0 = 0.0;
    double M0 = 0.0;

    bool newReading = false;
    bool reference_ = false;
    Attitude& attitude_;
    GNSSData& gnssData_;

    GnssSnapshot last_snapshot_;

    Eigen::Vector3f omega_ned_interp;
    Eigen::Vector3f r_ant_ned_interp;
    Eigen::Vector3f v_ant_ned;

    uint32_t t_GNSS_read_us = 0;
};

#endif
