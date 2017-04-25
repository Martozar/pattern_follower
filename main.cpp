#include "opencv2/features2d/features2d.hpp"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <pattern_follower/includes.h>
#include <queue>
#include <thread>
#include <time.h>

int main(int argc, char **argv) {
  Parser p(argc, argv);
  p.about("Application name v1.0.0\n");
  if (p.has("help")) {
    p.printMessage();
    return 0;
  }

  if (p.has("calib")) {
    run_calibration(p.get<String>("cp"));
    return 0;
  }

  bool simulation = p.has("simulation");

  String camera_file = p.get<String>("params");

  Mat grayImage, binaryImage, frame;

  Mat cameraMatrix;
  Mat distCoeffs;
  std::vector<int> labels;
  std::vector<Point> locations, approx;
  std::vector<std::vector<Point>> contours;
  std::vector<Vec4i> hierarchy;
  std::vector<Point2f> vertices;

  CCameraCalibration calibrator;

  calibrator.readFromFile(camera_file);

  cameraMatrix = calibrator.cameraMatrix;

  distCoeffs = calibrator.distortionCoefficients;

  double fovx = 2 * atan((FRAME_SIZE / 2 * cameraMatrix.at<double>(0)));
  Camera camera(FRAME_SIZE, NORM_PATTERN_SIZE / 100.0, 30, 155,
                detectorType::PF_TEMPLATE);

  double angle = 0.0;
  RobotControl rc(cv::Point2d(dist, angle), 1.0, simulation);

  while (true) {
    kf.prediction();
    if (camera.proceed(angle, dist)) {
      kf.update(dist, angle);
    } else {
      dist = kf.getX();
      angle = kf.getY();
    }

    if (!simulation) {
      rc.calculateRobotSpeeds(std::vector<cv::Point2d>(),
                              cv::Point2d(dist, angle));
    }
    if (waitKey(50) >= 0)
      break;
  }
}
