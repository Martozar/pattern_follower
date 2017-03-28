#ifndef UTILS_H
#define UTILS_H

#include <cmath>

double radToDeg(const double &angleInRad, const bool &positiveOnly = false);
double degToRad(const double &angleInDeg, const bool &positiveOnly = false);
double atanInPosDeg(const double &x, const double &y);
double degCoordToRadCoord(const double &angleInDeg);
double radCoordToDegCoord(const double &angleInRad);
int mod(const int &x, const int &y);

#endif
