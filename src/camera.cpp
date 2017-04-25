
#include <pattern_follower/camera.h>
#include <time.h> /* clock_t, clock, CLOCKS_PER_SEC */

Camera::Camera(const int &frameSize, const double &patternWidthCm,
               const int &distance, const int &patternWidthPix,
               const detectorType &detType, const int &port) {
  cap_(port);
  measurement_ = std::unique_ptr<Measurement>(
      new Measurement(frameSize, patternWidthCm, distance, patternWidthPix));
  kalmanFilter_ = std::unique_ptr<KalmanFilter_>();
  if (detType == detectorType::PF_TEMPLATE) {
    detector_ = std::unique_ptr<Detector>(new TemplateMatcher);
  } else {
    detector_ = std::unique_ptr<Detector>(new ArucoDetector);
  }
}

bool Camera::proceed(double &angle, double &distance) {
  cv::Mat image;
  cap_ >> image;
  return proceed(image, angle, distance);
}

bool Camera::proceed(cv::Mat &image, double &angle, double &distance) {

  clock_t t;

  t = clock();

  std::vector<std::vector<cv::Point2f>> cor;
  std::vector<int> ids;
  detector_->detect(image, cor, ids);
  t = clock() - t;
  printf("It took me %d clicks (%f seconds).\n", t,
         ((float)t) / CLOCKS_PER_SEC);
  std::cout << ids.size() << "\n";
  if (ids.size() > 0) {
    detector_->drawDetected(image, cor, ids);
    angle = measurement_->angle(cor.at(0));
    distance = measurement_->distance(cor.at(0));
  }
  imshow("Frame", image);

  return ids.size() > 0;
}
