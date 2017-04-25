#ifndef PID_H
#define PID_H

#include <chrono>
#include <fstream>
#include <iostream>

class PID {
public:
  PID(const double &Kp, const double &Ki, const double &Kd, const double &Ka,
      const double &max, const double &min, const double &eps);
  double calculate(const double &setPoint, const double &systemOutput);

protected:
private:
  double Kp_, Ki_, Kd_, Ka_, maxOut_, minOut_, eps_;
  double previousError_{0.0}, integral_{0.0}, privPresatOut_{0.0},
      privOut_{0.0};
  std::chrono::steady_clock::time_point lastUpdate_;
};

#endif // PID_H
