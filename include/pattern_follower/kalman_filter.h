#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <opencv2/opencv.hpp>
#include <chrono>

class KalmanFilter_ {
public:
  KalmanFilter_(const double &init_x, const double &init_y);

  void prediction(const double &forVel, const double &angVel);

  void update(const double &measured_x, const double &measured_y);

  const double &getX();
  const double &getY();

private:
  cv::Mat P, F, F_transp, H, H_transp, I, x, R;
  std::chrono::steady_clock::time_point lastUpdate; 
};
#endif
