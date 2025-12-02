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
            gnssData_.posN = (gnssData_.lat - gnssData_.lat0) * (M0 + gnssData_.alt0);
            gnssData_.posE = (gnssData_.lon - gnssData_.lon0) * (N0 + gnssData_.alt0) * cos(gnssData_.lat0);
            gnssData_.posD = (gnssData_.alt0 - gnssData_.alt);
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
    gnssData_.alt0 = alt0;
    N0 = a / sqrt(1 - e2 * sin(gnssData_.lat0) * sin(gnssData_.lat0));
    M0 = a * (1 - e2) / pow(1 - e2 * sin(gnssData_.lat0) * sin(gnssData_.lat0), 1.5);
    reference_ = true;
}

String GNSS::getDate() {
    String filename = String(ubx.utc_year()) + String("_") + String(ubx.utc_month()) + String("_") + String(ubx.utc_day()) + String("_") + String(ubx.utc_hour()) + String("-") + String(ubx.utc_min()) + String("-") + String(ubx.utc_sec()) + String(".txt");
    return filename;
}
