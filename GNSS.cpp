#include "GNSS.hpp"

void GNSS::setup() {
    Serial5.begin(115200);
    ubx_.Begin(115200);
}

void GNSS::read() {
    if (ubx_.Read()) {

        gnssData_.latGNSS = ubx_.lat_rad();
        gnssData_.lonGNSS = ubx_.lon_rad();
        gnssData_.altGNSS = ubx_.alt_wgs84_m();
        gnssData_.velNGNSS = ubx_.north_vel_mps();
        gnssData_.velEGNSS = ubx_.east_vel_mps();
        gnssData_.velDGNSS = ubx_.down_vel_mps();
        gnssData_.horAcc = ubx_.horz_acc_m();
        gnssData_.vertAcc = ubx_.vert_acc_m();
        gnssData_.numSV = ubx_.num_sv();
        gnssData_.fixType = static_cast<int>(ubx_.fix());


        if (!initialFix_ && f == 6) {
            gnssData_.lat0 = gnssData_.latGNSS;
            gnssData_.lon0 = gnssData_.lonGNSS;
            gnssData_.alt0 = gnssData_.altGNSS;
            N0 = a / sqrt(1 - e2 * sin(gnssData_.lat0) * sin(gnssData_.lat0));
            M0 = a * (1 - e2) / pow(1 - e2 * sin(gnssData_.lat0) * sin(gnssData_.lat0), 1.5);
            initialFix_ = true;
        }

        if (initialFix_) {
            gnssData_.n = (lat - gnssData_.lat0) * (gnssData_.M0 + gnssData_.alt0);
            gnssData_.e = (lon - gnssData_.lon0) * (gnssData_.N0 + gnssData_.alt0) * cos(gnssData_.lat0);
            gnssData_.d = (gnssData_.alt0 - alt);
        } else {
            gnssData_.n = 0;
            gnssData_.e = 0;
            gnssData_.d = 0;
        }
    }
}