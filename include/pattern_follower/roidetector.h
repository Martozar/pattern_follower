#ifndef ROIDETECTOR_H
#define ROIDETECTOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pattern_follower/contourfinder.h>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;

class RoiDetector {
public:
  RoiDetector(const double &_threshAdapt = 5, const int &_blockSize = 45,
              const int &_normSize = 80);
  virtual ~RoiDetector(){};
  void detectROI(const Mat &frame,
                 std::vector<std::vector<Point2f>> &refinedVertices,
                 std::vector<Mat> &regionsOfInterest);

protected:
private:
  int normSize;
  std::vector<Point2f> norm2dPRS;
  std::unique_ptr<ContourFinder> contourFinder;

  void normalizePattern(const Mat &src, const std::vector<Point2f> &roi,
                        Rect &rec, Mat &dst);
};

#endif // PATTERDETECTOR_H
