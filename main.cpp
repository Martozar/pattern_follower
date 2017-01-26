#include <pattern_follower/includes.h>

int main(int argc, char **argv) {
  Parser p(argc, argv, keys);
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
  String patterns = p.get<String>("p");

  Mat grayImage, binaryImage, frame;

  Mat cameraMatrix;
  Mat distCoeffs;

  std::vector<Mat> patternLibrary;
  std::vector<int> labels;
  std::vector<Pattern> detectedPattern;
  std::vector<Point> locations, approx;
  std::vector<std::vector<Point>> contours;
  std::vector<Vec4i> hierarchy;
  std::vector<Point2f> vertices;

  loadImages(patterns, patternLibrary);

  int norm_pattern_size = 80;
  int frame_size = 640;
  int adapt_block_size = 45;
  double adapt_thresh = 5;
  double confidenceThreshold = 0.9;

  if (patternLibrary.size() == 0)
    return -1;

  std::unique_ptr<Robot> r;
  if (simulation)
    r = std::make_unique<Robot>(MAX_VEL_SIM, MAX_ANG_VEL_SIM, ZRYCHLENI_SIM);
  else
    r = std::make_unique<Robot>(MAX_VEL_REAL, MAX_ANG_VEL_REAL, ZRYCHLENI_REAL,
                                IP, port);

  CCameraCalibration calibrator;

  calibrator.readFromFile(camera_file);

  cameraMatrix = calibrator.cameraMatrix;

  distCoeffs = calibrator.distortionCoefficients;

  double fovx = 2 * atan((2 * cameraMatrix.at<double>(0)) / frame_size);
  Camera camera(0, frame_size, 0.8, 30, 155, cameraMatrix, distCoeffs,
                "template", patternLibrary, confidenceThreshold);
  Measurement measurement(frame_size, 0.08, 30, 155, fovx);

  cv::Point2f q(255, 100);
  double q_x = 0.25;
  double q_y = 0.5;

  while (true) {

    double angle = 0.0;
    double dist = 80.0;
    // imshow("Copy", imcopy);
    camera.proceed(angle, dist);

    if (simulation) {
      Mat image = Mat::zeros(480, 640, CV_32F);
      cvtColor(image, image, CV_GRAY2BGR);
      circle(image, q, 8, Scalar(255, 0, 0), 2);
      line(image, Point2f(0, 480), Point2f(100, 480), Scalar(255, 0, 0), 10);
      line(image, Point2f(0, 480), Point2f(0, 380), Scalar(0, 0, 255), 10);

      angle = -atan2((q.y - (r->getY() + 50 * sin(r->getH()))),
                     (q.x - (r->getX() + 50 * cos(r->getH())))) +
              r->getH();
      dist = sqrt((q.x - (r->getX() + 50 * cos(r->getH()))) *
                      (q.x - (r->getX() + 50 * cos(r->getH()))) +
                  (q.y - (r->getY() + 50 * sin(r->getH()))) *
                      (q.y - (r->getY() + 50 * sin(r->getH()))));
      r->move_simulation(image, angle, dist);

      q.x += q_x;
      q.y += q_y;
      if (q.x > 640 || q.x < 0)
        q_x = -q_x;
      if (q.y > 480 || q.y < 0)
        q_y = -q_y;

      imshow("image", image);
    }

    else {
      r->move(angle, dist);
    }
    if (waitKey(50) >= 0)
      break;
  }
}
