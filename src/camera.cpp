#include <pattern_follower/camera.h>

Camera::Camera() : cap_(0) {}

Camera::Camera(const int &frameSize, const double &patternWidthCm,
               const int &distance, const int &patternWidthPix)
    : Camera() {
  measurement_ = std::unique_ptr<Measurement>(
      new Measurement(frameSize, patternWidthCm, distance, patternWidthPix));
  detector_ = std::unique_ptr<Detector>(new ArucoDetector);
}

Camera::Camera(const int &frameSize, const double &patternWidthCm,
               const int &distance, const int &patternWidthPix,
               const Mat &cameraMatrix, const Mat &distortions,
               const detectorType &detType)
    : Camera(frameSize, patternWidthCm, distance, patternWidthPix) {

  /*cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);*/
  if (detType == detectorType::PF_TEMPLATE)
    detector_ = std::unique_ptr<Detector>(
        new TemplateMatcher(cameraMatrix, distortions));
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
