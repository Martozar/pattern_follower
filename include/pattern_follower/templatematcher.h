#ifndef TEMPLATEMATCHER_H
#define TEMPLATEMATCHER_H

#include <algorithm>
#include <chrono>
#include <iostream>
#include <pattern_follower/contourfinder.h>
#include <pattern_follower/detector.h>
#include <pattern_follower/loader.h>
#include <pattern_follower/roidetector.h>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;

const int NORM_PATTERN_SIZE = 80;
const int BLOCK_SIZE = 45;
const double CONF_TRESH = 0.75;
const double ADAPT_THRESHOLD = 5;

class TemplateMatcher : public Detector {
public:
  TemplateMatcher(
      const double &confThreshold = CONF_TRESH,
      const int &normSize = NORM_PATTERN_SIZE,
      const cv::String &path = "/home/michail/pattern_follower/patterns/*.png",
      const double &adaptThreshold = ADAPT_THRESHOLD,
      const int &blockSize = BLOCK_SIZE);

  virtual void detect(Mat &frame, std::vector<std::vector<Point2f>> &corners,
                      std::vector<int> &ids) override;
  virtual void drawDetected(cv::Mat &frame,
                            std::vector<std::vector<cv::Point2f>> &corners,
                            std::vector<int> &ids) override;

protected:
private:
  struct patInfo {
    int index;
    int ori;
    double maxCor;
  };

  int normSize_;
  double confThreshold_;
  std::vector<Mat> library_;
  std::unique_ptr<RoiDetector> roiDetector_;

  bool identifyPattern(const Mat &src, patInfo &out);
  double correlation(Mat &image_1, Mat &image_2);
};

#endif // TAMPLATEMATCHER_H
