#ifndef PID_H
#define PID_H

#include <chrono>
#include <fstream>
#include <iostream>

class PID {
public:
  PID(const double &_Kp, const double &_Ki, const double &_Kd,
      const double &_Ka, const double &_max, const double &_min,
      const double &_eps)
      : Kp(_Kp), Ki(_Ki), Kd(_Kd), Ka(_Ka), max_out(_max), min_out(_min),
        previousError(0), integral(0) {
    privPresatOut = 0;
    privOut = 0;
    eps = _eps;
    lastUpdate = std::chrono::steady_clock::now();
  };
  virtual ~PID(){};
  double calculate(const double &setPoint, const double &systemOutput);

protected:
private:
  double Kp, Ki, Kd, Ka, max_out, min_out, eps;
  double previousError, integral, privPresatOut, privOut;
  std::chrono::steady_clock::time_point lastUpdate;
};

#endif // PID_H
