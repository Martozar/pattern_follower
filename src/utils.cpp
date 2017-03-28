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

double atanInPosDeg(const double &x, const double &y) {
  return -(std::atan2(x, y) / M_PI * 180.0 - 180.0);
}

double degCoordToRadCoord(const double &angleInDeg) {
  double angleInRad = degToRad(angleInDeg);
  if (angleInRad > 3.0 * M_PI / 2.0)
    angleInRad = 5.0 * M_PI / 2.0 - angleInRad;
  else
    angleInRad = M_PI / 2.0 - angleInRad;
  return angleInRad;
}
double radCoordToDegCoord(const double &angleInRad) {
  double angleInDeg = radToDeg(angleInRad);
  if (angleInDeg < -90)
    angleInDeg = 450.0 - angleInDeg;
  else
    angleInDeg = 90.0 - angleInDeg;
  return angleInDeg;
}
