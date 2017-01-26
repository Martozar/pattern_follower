#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <pattern_follower/arucodetector.h>
#include <pattern_follower/cam_calibration.h>
#include <pattern_follower/camera_calibration.h>
#include <pattern_follower/contourfinder.h>
#include <pattern_follower/measurement.h>
#include <pattern_follower/pattern.h>
#include <pattern_follower/roidetector.h>
#include <pattern_follower/templatematcher.h>

class Camera {
public:
  Camera(const int &cameraPort, const int &frame_size,
         const double &pattern_width_cm, const int &distance,
         const int &pattern_width_pix, const Mat &_cameraMatrix = Mat(),
         const Mat &_distortions = Mat(),
         const std::string &detectorType = "aruco",
         const std::vector<Mat> &_library = std::vector<Mat>(),
         const double &_confThreshold = 0.0);
  void proceed(double &angle, double &distance);
  void proceed(cv::Mat &image, double &angle, double &distance);

private:
  Measurement measurement;
  Detector *detector;
  cv::VideoCapture cap;
};
#endif
