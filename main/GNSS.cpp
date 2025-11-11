#include "GNSS.hpp"

bfs::Ubx ubx(&Serial5);

void GNSS::setup() {
  Serial5.begin(115200);
  ubx.Begin(115200);
}

bool GNSS::read() {
    newReading = ubx.Read();
    
    if (newReading) {
        gnssData_.latGNSS = ubx.lat_rad();
        gnssData_.lonGNSS = ubx.lon_rad();
        gnssData_.altGNSS = ubx.alt_wgs84_m();
        gnssData_.velNGNSS = ubx.north_vel_mps();
        gnssData_.velEGNSS = ubx.east_vel_mps();
        gnssData_.velDGNSS = ubx.down_vel_mps();
        gnssData_.horAcc = ubx.horz_acc_m();
        gnssData_.vertAcc = ubx.vert_acc_m();
        gnssData_.numSV = ubx.num_sv();
        gnssData_.fixType = static_cast<int>(ubx.fix());

        if (!reference_ && gnssData_.fixType == 6) {
            setReference(gnssData_.latGNSS, gnssData_.lonGNSS, gnssData_.altGNSS);
        }
        
        if (reference_) {
            gnssData_.posNGNSS = (gnssData_.latGNSS - gnssData_.lat0) * (M0 + gnssData_.alt0);
            gnssData_.posEGNSS = (gnssData_.lonGNSS - gnssData_.lon0) * (N0 + gnssData_.alt0) * cos(gnssData_.lat0);
            gnssData_.posDGNSS = (gnssData_.alt0 - gnssData_.altGNSS);
        } else {
            gnssData_.posNGNSS = 0;
            gnssData_.posEGNSS = 0;
            gnssData_.posDGNSS = 0;
        }
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