#ifndef DETECTOR_H
#define DETECTOR_H
#include <opencv2/opencv.hpp>
class Detector {
public:
  virtual void detect(cv::Mat &frame,
                      std::vector<std::vector<cv::Point2f>> &corners,
                      std::vector<int> &ids) = 0;

  virtual void drawDetected(cv::Mat &frame,
                            std::vector<std::vector<cv::Point2f>> &corners,
                            std::vector<int> &ids) = 0;
};
#endif
