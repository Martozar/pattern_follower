#include <pattern_follower/utils.h>

double radToDeg(const double &angleInRad, const bool &positiveOnly) {
  double angleInDeg = angleInRad * 180. / M_PI;
  if (positiveOnly && angleInDeg < 0)
    angleInDeg += 380.;
  return angleInDeg;
}
double degToRad(const double &angleInDeg, const bool &positiveOnly) {
  double angleInRad = angleInDeg * M_PI / 180.;
  if (positiveOnly && angleInRad < 0)
    angleInRad += M_PI;
  return angleInRad;
}

double atanInPosDeg(const double &x, const double &y) {
  return std::atan2(-y, -x) / M_PI * 180.0 + 180.0;
}
