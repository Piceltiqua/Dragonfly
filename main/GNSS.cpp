#include "GNSS.hpp"

bfs::Ubx ubx(&Serial5);

void GNSS::setup() {
    Serial5.begin(115200);
    ubx.Begin(115200);
}

bool GNSS::read() {
    newReading = ubx.Read();

    if (newReading) {
        t_GNSS_read_us = micros() - GNSS_VEL_LATENCY_US;
        gnssData_.lat = ubx.lat_rad();
        gnssData_.lon = ubx.lon_rad();
        gnssData_.alt = ubx.alt_wgs84_m();
        v_ant_ned.x() = ubx.north_vel_mps();
        v_ant_ned.y() = ubx.east_vel_mps();
        v_ant_ned.z() = ubx.down_vel_mps();
        gnssData_.horAcc = ubx.horz_acc_m();
        gnssData_.vertAcc = ubx.vert_acc_m();
        gnssData_.numSV = ubx.num_sv();
        // Serial.print("GNSS SPEED ACCURACY : ");
        // Serial.println(ubx.spd_acc_mps(), 4);
        gnssData_.fixType = static_cast<int>(ubx.fix());

        if (!reference_ && gnssData_.fixType == 6) {
            setReference(gnssData_.lat, gnssData_.lon, gnssData_.alt);
        }

        if (reference_) {
            computeCGPositionNED();
            computeCGVelocityNED();
        } else {
            gnssData_.posN = 0;
            gnssData_.posE = 0;
            gnssData_.posD = 0;
        }
    }
    return newReading;
}

void GNSS::setReference(double lat0, double lon0, double alt0) {
    gnssData_.lat0 = lat0;
    gnssData_.lon0 = lon0;
    gnssData_.alt0 = alt0 + r_ant_cad.z();
    N0 = a / sqrt(1 - e2 * sin(gnssData_.lat0) * sin(gnssData_.lat0));
    M0 = a * (1 - e2) / pow(1 - e2 * sin(gnssData_.lat0) * sin(gnssData_.lat0), 1.5);
    reference_ = true;
}

void GNSS::computeCGPositionNED() {
    Eigen::Quaternionf q_cad_to_ned(attitude_.qw, attitude_.qi, attitude_.qj, attitude_.qk);
    q_cad_to_ned.normalize();

    // Antenna position offset in NED frame
    Eigen::Vector3f r_ant_ned = q_cad_to_ned * r_ant_cad;

    // Measured antenna position in NED frame
    float posN_ant = (gnssData_.lat - gnssData_.lat0) * (M0 + gnssData_.alt0);
    float posE_ant = (gnssData_.lon - gnssData_.lon0) * (N0 + gnssData_.alt0) * cos(gnssData_.lat0);
    float posD_ant = (gnssData_.alt0 - gnssData_.alt);
    Eigen::Vector3f p_ant_ned(posN_ant, posE_ant, posD_ant);

    // CG position in NED frame
    Eigen::Vector3f p_cg_ned = p_ant_ned - r_ant_ned;
    gnssData_.posN = p_cg_ned.x();
    gnssData_.posE = p_cg_ned.y();
    gnssData_.posD = p_cg_ned.z();

    // Store snapshot for logging
    last_snapshot_.r_ant_ned_x = r_ant_ned.x();
    last_snapshot_.r_ant_ned_y = r_ant_ned.y();
    last_snapshot_.r_ant_ned_z = r_ant_ned.z();
    last_snapshot_.p_ant_ned_x = p_ant_ned.x();
    last_snapshot_.p_ant_ned_y = p_ant_ned.y();
    last_snapshot_.p_ant_ned_z = p_ant_ned.z();
    last_snapshot_.p_cg_ned_x = p_cg_ned.x();
    last_snapshot_.p_cg_ned_y = p_cg_ned.y();
    last_snapshot_.p_cg_ned_z = p_cg_ned.z();
}

void GNSS::computeCGVelocityNED() {
    // Obtain omega_ned and r_ant_ned at t_GNSS_read_us via interpolation
    if (!interpolateIMUSample()) {
        Serial.print("GNSS::computeCGVelocityNED(): interpolation failed\n");
    }

    // CG velocity in NED frame by subtracting the rotational component (cross product) from the total antenna velocity to obtain the velocity due to translation only
    Eigen::Vector3f v_cg_ned = v_ant_ned - omega_ned_interp.cross(r_ant_ned_interp);

    // Store results with exponential smoothing
    gnssData_.velN = alpha * v_cg_ned.x() + (1 - alpha) * gnssData_.velN;
    gnssData_.velE = alpha * v_cg_ned.y() + (1 - alpha) * gnssData_.velE;
    gnssData_.velD = alpha * v_cg_ned.z() + (1 - alpha) * gnssData_.velD;
    // Store snapshot for logging
    last_snapshot_.v_cg_ned_x = v_cg_ned.x();
    last_snapshot_.v_cg_ned_y = v_cg_ned.y();
    last_snapshot_.v_cg_ned_z = v_cg_ned.z();
    last_snapshot_.v_ant_ned_x = v_ant_ned.x();
    last_snapshot_.v_ant_ned_y = v_ant_ned.y();
    last_snapshot_.v_ant_ned_z = v_ant_ned.z();
    last_snapshot_.omega_ned_interp_x = omega_ned_interp.x();
    last_snapshot_.omega_ned_interp_y = omega_ned_interp.y();
    last_snapshot_.omega_ned_interp_z = omega_ned_interp.z();
    last_snapshot_.r_ant_ned_interp_x = r_ant_ned_interp.x();
    last_snapshot_.r_ant_ned_interp_y = r_ant_ned_interp.y();
    last_snapshot_.r_ant_ned_interp_z = r_ant_ned_interp.z();
}

bool GNSS::interpolateIMUSample() {
    if (imu_buf.size() < 2) return false;

    // find two samples bracketing t_GNSS_read_us
    size_t i = 0;  // Index of earlier sample
    while (i + 1 < imu_buf.size() && imu_buf[i + 1].t_us < (t_GNSS_read_us)) {
        ++i;
    }

    // Serial.println("1");
    if (i + 1 >= imu_buf.size()) return false;  // requested time too new

    const ImuSample& sampleBefore = imu_buf[i];
    const ImuSample& sampleAfter = imu_buf[i + 1];

    if (t_GNSS_read_us < sampleBefore.t_us || t_GNSS_read_us > sampleAfter.t_us) return false;
    double dtAB = double(sampleAfter.t_us - sampleBefore.t_us);

    // Serial.println("3");
    if (dtAB <= 0.0) return false;
    double alpha = double(t_GNSS_read_us - sampleBefore.t_us) / dtAB;

    // linear interpolate omega_ned and r_ant_ned
    omega_ned_interp = (1.0 - alpha) * sampleBefore.omega_NED + alpha * sampleAfter.omega_NED;
    r_ant_ned_interp = (1.0 - alpha) * sampleBefore.r_ant_ned + alpha * sampleAfter.r_ant_ned;

    return true;
}

String GNSS::getDateFilename() {
    String filename;
    if (gnssData_.fixType < 3) {
        // No valid fix, return default filename
        filename = String("NO_FIX_LOG_") + String(micros()) + String(".txt");
    } else {
        // Valid fix, use date and time from GNSS
        filename = String(ubx.utc_year()) + String("_") + String(ubx.utc_month()) + String("_") + String(ubx.utc_day()) + String("_") + String(ubx.utc_hour()) + String("-") + String(ubx.utc_min()) + String("-") + String(ubx.utc_sec()) + String(".txt");
    }
    return filename;
}