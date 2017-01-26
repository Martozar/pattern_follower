#ifndef ROIDETECTOR_H
#define ROIDETECTOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;

class RoiDetector {
public:
  RoiDetector(const double &_threshAdapt = 5, const int &_blockSize = 45,
              const int &_normSize = 80);
  virtual ~RoiDetector(){};
  void detectROI(const std::vector<Point> &contours,
                 const std::vector<Vec4i> &hierarchy, const int index,
                 const Mat &grayImage, std::vector<Point2f> &refinedVertices,
                 int &vertex);
  Mat &getNormROI() { return normROI; };

protected:
private:
  int normSize, blockSize;
  double threshAdapt;
  Mat normROI;
  Point2f norm2dPRS[4];

  void binarize(const Mat &src, Mat &image_gray, Mat &dst);
  void normalizePattern(const Mat &src, const Point2f roi[], Rect &rec,
                        Mat &dst);
};

#endif // PATTERDETECTOR_H
