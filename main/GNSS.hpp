// GNSS.hpp
#ifndef GNSS_H
#define GNSS_H

#include "ubx.h"
#include "Utils.hpp"

extern bfs::Ubx ubx;

class GNSS {
public:
  GNSS(GNSSData &gnssData)
  : gnssData_(gnssData) {}
  
  void setup();
  bool read();
  void setReference(double lat0, double lon0, double alt0);

private:
  static constexpr double a = 6378137.0;
  static constexpr double f = 1.0 / 298.257223563;
  static constexpr double e2 = f * (2.0 - f);

  double N0 = 0.0;
  double M0 = 0.0;

  bool newReading = false;
  bool reference_ = false;
  GNSSData& gnssData_;
};

#endif
