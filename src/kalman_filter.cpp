#include <iostream>
#include <pattern_follower/kalman_filter.h>

KalmanFilter_::KalmanFilter_(const double &init_x, const double &init_y) {
  // F = [1 0 dt 0;
  //      0  1 0 dt;
  //      0  0 1 0;
  //      0  0 0 1]
  F = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  cv::transpose(F, F_transp);
  P = cv::Mat::eye(4, 4, CV_64FC1) * 1000.0;
  H = (cv::Mat_<double>(2, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  cv::transpose(H, H_transp);
  R = cv::Mat::eye(2, 2, CV_64FC1) * 1e-2;
  I = cv::Mat::eye(4, 4, CV_64FC1);
  x = (cv::Mat_<double>(4, 1) << init_x, init_y, 0.0, 0.0);
  Q = cv::Mat::eye(4, 4, CV_64FC1) * 1e-2;
  std::cout << Q << "\n";
  B = (cv::Mat_<double>(4, 4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
       1) *
      1e-3;

  std::cout << B << "\n";
  lastUpdate = std::chrono::steady_clock::now();
}

// TODO: add robot change position to prediction
void KalmanFilter_::prediction() {
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff = now - lastUpdate;
  double dt = diff.count();
  lastUpdate = now;
  F.at<double>(0, 2) = dt;
  F.at<double>(1, 3) = dt;
  Q.at<double>(0, 0) = dt * dt * dt / 3.0;
  Q.at<double>(0, 2) = dt * dt / 2.0;
  Q.at<double>(1, 1) = dt * dt * dt / 3.0;
  Q.at<double>(1, 3) = dt * dt / 2.0;
  Q.at<double>(2, 0) = dt * dt / 2.0;
  Q.at<double>(2, 2) = dt / 3.0;
  Q.at<double>(3, 1) = dt * dt / 2.0;
  Q.at<double>(3, 3) = dt / 3.0;
  x = (F - B) * x;
  std::cout << x << "\n";
  P = F * P * F_transp + Q;
}

void KalmanFilter_::update(const double &measured_x, const double &measured_y) {
  cv::Mat Y = (cv::Mat_<double>(2, 1) << measured_x, measured_y) - H * x;

  cv::Mat S = H * P * H_transp + R;
  cv::Mat K = P * H_transp * S.inv();
  x = x + K * Y;
  P = (I - K * H) * P;
}

const double &KalmanFilter_::getX() { return x.at<double>(0); }
const double &KalmanFilter_::getY() { return x.at<double>(1); }
