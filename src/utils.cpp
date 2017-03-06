#include <iostream>
#include <pattern_follower/utils.h>

double radToDeg(const double &angleInRad, const bool &positiveOnly) {
  double angleInDeg = angleInRad * 180.0 / M_PI;
  if (positiveOnly && angleInDeg < 0)
    angleInDeg += 360.0;
  return angleInDeg;
}

double degToRad(const double &angleInDeg, const bool &positiveOnly) {
  double angleInRad = angleInDeg * M_PI / 180.;
  if (positiveOnly && angleInRad < 0.0)
    angleInRad += 2.0 * M_PI;
  return angleInRad;
}

double atanInPosDeg(const double &y, const double &x) {
  return -(std::atan2(y, x) / M_PI * 180.0 - 180.0);
}
