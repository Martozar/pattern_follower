#ifndef TEMPLATEMATCHER_H
#define TEMPLATEMATCHER_H

#include <algorithm>
#include <iostream>
#include <pattern_follower/contourfinder.h>
#include <pattern_follower/detector.h>
#include <pattern_follower/roidetector.h>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;

class TemplateMatcher : public Detector {
public:
  TemplateMatcher(const std::vector<Mat> &_library,
                  const double &_confThreshold, const int &_normSize,
                  const Mat &_cameraMatrix, const Mat &_distortions)
      : contourFinder(), roiDetector() {
    library = _library;
    confThreshold = _confThreshold;
    normSize = _normSize;
    cameraMatrix = _cameraMatrix;
    distortions = _distortions;
  };
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
  int normSize;
  double confThreshold;
  std::vector<Mat> library;
  ContourFinder contourFinder;
  RoiDetector roiDetector;
  Mat distortions, cameraMatrix;
  bool identifyPattern(const Mat &src, patInfo &out);
  double correlation(Mat &image_1, Mat &image_2);
};

#endif // TAMPLATEMATCHER_H
