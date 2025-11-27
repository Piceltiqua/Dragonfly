#include "GNSS.hpp"

bfs::Ubx ubx(&Serial5);

void GNSS::setup() {
  Serial5.begin(115200);
  ubx.Begin(115200);
}

bool GNSS::read() {
  newReading = ubx.Read();

  if (newReading) {
    gnssData_.lat = ubx.lat_rad();
    gnssData_.lon = ubx.lon_rad();
    gnssData_.alt = ubx.alt_wgs84_m();
    gnssData_.velN = ubx.north_vel_mps();
    gnssData_.velE = ubx.east_vel_mps();
    gnssData_.velD = ubx.down_vel_mps();
    gnssData_.horAcc = ubx.horz_acc_m();
    gnssData_.vertAcc = ubx.vert_acc_m();
    gnssData_.numSV = ubx.num_sv();
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
    Serial.print(millis());
    Serial.print(",");
    Serial.print(gnssData_.posN);
    Serial.print(",");
    Serial.print(gnssData_.posE);
    Serial.print(",");
    Serial.print(gnssData_.posD);
    Serial.print(",");
    Serial.print(gnssData_.velN);
    Serial.print(",");
    Serial.print(gnssData_.velE);
    Serial.print(",");
    Serial.print(gnssData_.velD);
    Serial.print(",");
    Serial.print(gnssData_.horAcc);
    Serial.print(",");
    Serial.print(gnssData_.vertAcc);
    Serial.print(",");
    Serial.print(gnssData_.numSV);
    Serial.print(",");
    Serial.println(gnssData_.fixType);
  }
  return newReading;
}

void GNSS::setReference(double lat0, double lon0, double alt0) {
  gnssData_.lat0 = lat0;
  gnssData_.lon0 = lon0;
  gnssData_.alt0 = alt0;
  N0 = a / sqrt(1 - e2 * sin(gnssData_.lat0) * sin(gnssData_.lat0));
  M0 = a * (1 - e2) / pow(1 - e2 * sin(gnssData_.lat0) * sin(gnssData_.lat0), 1.5);
  reference_ = true;
}

void GNSS::computeCGPositionNED() {
  Eigen::Quaternionf q_cad_to_ned(attitude_.qw, attitude_.qi, attitude_.qj, attitude_.qk);
  q_cad_to_ned.normalize();
  Eigen::Vector3f offset_ned = q_cad_to_ned * p_ant_cad;

  float posN_ant = (gnssData_.lat - gnssData_.lat0) * (M0 + gnssData_.alt0);
  float posE_ant = (gnssData_.lon - gnssData_.lon0) * (N0 + gnssData_.alt0) * cos(gnssData_.lat0);
  float posD_ant = (gnssData_.alt0 - gnssData_.alt);

  Eigen::Vector3f p_ant_ned(posN_ant, posE_ant, posD_ant);

  Eigen::Vector3f p_cg_ned = p_ant_ned - offset_ned;

  // Position of the center of gravity in NED frame
  gnssData_.posN = p_cg_ned.x();
  gnssData_.posE = p_cg_ned.y();
  gnssData_.posD = p_cg_ned.z();
}

void GNSS::computeCGVelocityNED() {
  Eigen::Quaternionf q_cad_to_ned(attitude_.qw, attitude_.qi, attitude_.qj, attitude_.qk);
  q_cad_to_ned.normalize();

  // IMU angular rates in IMU frame
  Eigen::Vector3f omega_IMU(attitude_.wx, attitude_.wy, attitude_.wz);

  // Angular rates in CAD frame
  Eigen::Vector3f omega_cad = q_cad_to_imu.conjugate() * omega_IMU;

  // Angular rates in NED frame
  Eigen::Vector3f omega_ned = q_cad_to_ned * omega_cad;

  // Antenna position in NED frame
  Eigen::Vector3f p_ant_ned = q_cad_to_ned * p_ant_cad;

  // Measured antenna velocity in NED frame
  Eigen::Vector3f v_ant_ned(gnssData_.velN, gnssData_.velE, gnssData_.velD);

  // CG velocity in NED frame
  Eigen::Vector3f v_cg_ned = v_ant_ned - omega_ned.cross(p_ant_ned);

  // Store results
  gnssData_.velN = v_cg_ned.x();
  gnssData_.velE = v_cg_ned.y();
  gnssData_.velD = v_cg_ned.z();
}