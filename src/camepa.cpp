#include <pattern_follower/camera.h>

Camera::Camera(const int &cameraPort, const int &frame_size,
               const double &pattern_width_cm, const int &distance,
               const int &pattern_width_pix, const Mat &_cameraMatrix,
               const Mat &_distortions, const std::string &detectorType,
               const std::vector<Mat> &_library, const double &_confThreshold)
    : measurement(frame_size, pattern_width_cm, distance, pattern_width_pix),
      cap(cameraPort) {

  /*cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);*/
  if (detectorType == "aruco")
    detector = new ArucoDetector();
  else if (detectorType == "template")
    detector =
        new TemplateMatcher(_library, _confThreshold, pattern_width_cm * 100,
                            _cameraMatrix, _distortions);
}

void Camera::proceed(double &angle, double &distance) {
  cv::Mat image;
  cap >> image;
  proceed(image, angle, distance);
}

void Camera::proceed(cv::Mat &image, double &angle, double &distance) {
  std::vector<std::vector<cv::Point2f>> cor;
  std::vector<int> ids;
  detector->detect(image, cor, ids);
  if (ids.size() > 0) {
    detector->drawDetected(image, cor, ids);
    angle = measurement.angle(cor.at(0));
    distance = measurement.distance(cor.at(0));
  }
  imshow("Frame", image);
}
