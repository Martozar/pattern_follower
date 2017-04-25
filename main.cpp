#include "opencv2/features2d/features2d.hpp"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <pattern_follower/includes.h>
#include <queue>
#include <thread>
#include <time.h>

int main(int argc, char **argv) {
  FileStorage fs("/home/michail/pattern_follower/config.yaml",
                 FileStorage::READ);

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
  Camera c1(fs["Camera"]);

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

  double angle = 0.0;
  double dist = 80;
  RobotControl rc(cv::Point2d(dist, angle), 1.0, simulation);

  /*while (true) {
    if (camera.proceed(angle, dist)) {
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
  }*/
}
