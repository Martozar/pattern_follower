#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "opencv2/core/core.hpp"
#include <chrono>

class KalmanFilter_ {
public:
  KalmanFilter_(const cv::FileNode &fn);

  void prediction();

  void update(const double &measured_x, const double &measured_y);

  const double &getX();
  const double &getY();

private:
  cv::Mat P, F, F_transp, H, H_transp, I, x, R, Q, B;
  std::chrono::steady_clock::time_point lastUpdate;
};
#endif
