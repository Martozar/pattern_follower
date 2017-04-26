#ifndef CONTOURFINDER_H
#define CONTOURFINDER_H

#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
class ContourFinder {
public:
  ContourFinder(const int &_threshold = 5, const int &_blockSize = 45)
      : threshold(_threshold), blockSize(_blockSize){};
  const int getThreshold() const { return threshold; };
  void setThreshold(const int &_threshold) { threshold = _threshold; };
  void contours(const Mat &image, std::vector<std::vector<Point>> &contours,
                std::vector<Vec4i> &hierarchy) const;
  void binarize(const Mat &image, Mat &grayImage, Mat &binaryImage);

private:
  int threshold;
  int blockSize;
};

#endif // CONTOURFINDER_H
