
#include <pattern_follower/camera.h>
#include <time.h> /* clock_t, clock, CLOCKS_PER_SEC */

Camera::Camera(const FileNode &fn) {
  cap_ = cv::VideoCapture((int)fn["port"]);
  measurement_ =
      std::unique_ptr<Measurement>(new Measurement(fn["Measurement"]));
  if (fn["detector_type"] == "aruco") {
    detector_ = std::unique_ptr<Detector>(new ArucoDetector);
  } else {
    detector_ =
        std::unique_ptr<Detector>(new TemplateMatcher(fn["TemplateMatcher"]));
  }
}

bool Camera::proceed(double &angle, double &distance) {
  cv::Mat image;
  cap_ >> image;
  return proceed(image, angle, distance);
}

bool Camera::proceed(cv::Mat &image, double &angle, double &distance) {
  std::vector<std::vector<cv::Point2f>> cor;
  std::vector<int> ids;
  detector_->detect(image, cor, ids);
  if (ids.size() > 0) {
    detector_->drawDetected(image, cor, ids);
    angle = measurement_->angle(cor.at(0));
    distance = measurement_->distance(cor.at(0));
  }

  imshow("Frame", image);
  return ids.size() > 0;
}
