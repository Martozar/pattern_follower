/**
 * @file contourfinder.h
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

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

  /**
   * Finds contours in image.
   * @param [in] image
   * @param [out] contours found contours in image.
   * @param [out] hierarchy contours' hierarchy with CV_RETR_TREE flag.
   */
  void contours(const Mat &image, std::vector<std::vector<Point>> &contours,
                std::vector<Vec4i> &hierarchy) const;
  /**
   * Binarize image using adaptive threshold.
   * @param [in] image RGB or gray image
   * @param [out] grayImage
   * @param [out] binaryImage thresholded image.
   */
  void binarize(const Mat &image, Mat &grayImage, Mat &binaryImage);

private:
  int threshold;
  int blockSize;
};

#endif // CONTOURFINDER_H
