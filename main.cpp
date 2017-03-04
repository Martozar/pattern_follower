#include "opencv2/features2d/features2d.hpp"
#include <pattern_follower/includes.h>
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

  std::unique_ptr<Robot> r;
  if (simulation)
    r = std::make_unique<Robot>(cv::Point2d(80.0, 0.0), MAX_VEL_SIM,
                                MAX_ANG_VEL_SIM, ZRYCHLENI_SIM);
  else
    r = std::make_unique<Robot>(cv::Point2d(80.0, 0.0), MAX_VEL_REAL,
                                MAX_ANG_VEL_REAL, ZRYCHLENI_REAL, IP, port);

  CCameraCalibration calibrator;

  calibrator.readFromFile(camera_file);

  cameraMatrix = calibrator.cameraMatrix;

  distCoeffs = calibrator.distortionCoefficients;

  double fovx = 2 * atan((FRAME_SIZE / 2 * cameraMatrix.at<double>(0)));
  Camera camera(FRAME_SIZE, NORM_PATTERN_SIZE / 100.0, 30, 155);
  Measurement measurement(FRAME_SIZE, NORM_PATTERN_SIZE / 100.0, 30, 155, fovx);
  KalmanFilter_ kf(80.0, 0.0);

  double angle = 0.0;
  double dist = 80.0;

  while (true) {
    kf.prediction(0.0, 0.0);
    if (camera.proceed(angle, dist)) {
      kf.update(dist, angle);
    } else {
      dist = kf.getX();
      angle = kf.getY();
    }

    if (!simulation) {
      r->move(angle, dist);
    }
    if (waitKey(50) >= 0)
      break;
  }
}
