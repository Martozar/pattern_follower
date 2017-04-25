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

class TemplateMatcher : public Detector {
public:
  TemplateMatcher(const FileNode &fn);

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
