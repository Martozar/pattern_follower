#ifndef PID_H
#define PID_H

#include "opencv2/core/persistence.hpp"
#include <chrono>
#include <fstream>
#include <iostream>

class PID {
public:
  PID(const cv::FileNode &fn, const double &max, const double &min);

  double calculate(const double &setPoint, const double &systemOutput);

protected:
private:
  double Kp_, Ki_, Kd_, Ka_, maxOut_, minOut_, eps_;
  double previousError_{0.0}, integral_{0.0}, privPresatOut_{0.0},
      privOut_{0.0};
  std::chrono::steady_clock::time_point lastUpdate_;
};

#endif // PID_H
