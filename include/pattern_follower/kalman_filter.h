/**
 * @file kalman_filter.h
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "opencv2/core/core.hpp"
#include <chrono>

class KalmanFilter_ {
public:
  KalmanFilter_(const cv::FileNode &fn);

  const double &getX() { return x.at<double>(0); }
  const double &getY() { return x.at<double>(1); }

  /**
   * Perform prediction step of KF.
   */
  void prediction();

  /**
   * Perform update step of KF.
   * @param [in] measured_x measured target x coordinate.
   * @param [in] measured_y measured target y coordinate.
   */
  void update(const double &measured_x, const double &measured_y);

private:
  cv::Mat P, F, F_transp, H, H_transp, I, x, R, Q, B;
  std::chrono::steady_clock::time_point lastUpdate;
};
#endif
