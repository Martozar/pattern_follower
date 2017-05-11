/**
 * @file roidetector.h
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

#ifndef ROIDETECTOR_H
#define ROIDETECTOR_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pattern_follower/contourfinder.h>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;

class RoiDetector {
public:
  RoiDetector(const double &_threshAdapt, const int &_blockSize,
              const int &_normSize);
  /**
   * Find regions of interest in frame.
   *
   * @param [in] frame input fream to detect roi in.
   * @param [out] refinedVertices found vertices of ROI.
   * @param [out] regionsOfInterest found regiond of interest.
   */
  void detectROI(const Mat &frame,
                 std::vector<std::vector<Point2f>> &refinedVertices,
                 std::vector<Mat> &regionsOfInterest);

private:
  int normSize;
  Mat normROI_;
  std::vector<Point2f> norm2dPRS;
  std::unique_ptr<ContourFinder> contourFinder;

  void normalizePattern(const Mat &src, const std::vector<Point2f> &roi,
                        Rect &rec, Mat &dst);
};

#endif // PATTERDETECTOR_H
