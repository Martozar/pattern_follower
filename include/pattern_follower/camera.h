#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <pattern_follower/arucodetector.h>
#include <pattern_follower/cam_calibration.h>
#include <pattern_follower/camera_calibration.h>
#include <pattern_follower/contourfinder.h>
#include <pattern_follower/kalman_filter.h>
#include <pattern_follower/measurement.h>
#include <pattern_follower/roidetector.h>
#include <pattern_follower/templatematcher.h>

const int PORT = 0;
class Camera {
public:
  Camera(const FileNode &fn);
  bool proceed(double &angle, double &distance);

  bool proceed(cv::Mat &image, double &angle, double &distance);

private:
  std::unique_ptr<Measurement> measurement_;
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<KalmanFilter_> kalmanFilter_;
  cv::VideoCapture cap_;
};
#endif
