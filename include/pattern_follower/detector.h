/**
 * @file detector.h
 * @breaf Purpose: abstract class to inherit detector from.
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

#ifndef DETECTOR_H
#define DETECTOR_H
#include <opencv2/opencv.hpp>

enum class detectorType { PF_TEMPLATE, PF_ARUCO };
class Detector {
public:
  /**
   * Detects markers from detector's library in camera frame.
   * @param [in] frame output camera frame
   * @param [out] corners positions of markers' corners in the frame
   * @param [out] ids indexes of markers from detector's library
   */
  virtual void detect(cv::Mat &frame,
                      std::vector<std::vector<cv::Point2f>> &corners,
                      std::vector<int> &ids) = 0;

  /**
   * Draws detected markers to frame
   * @param [int-out] frame output camera frame (input-output parameter)
   * @param [in] corners positions of markers' corners in the frame
   * @param [in] ids indexes of markers from detector's library
   */
  virtual void drawDetected(cv::Mat &frame,
                            std::vector<std::vector<cv::Point2f>> &corners,
                            std::vector<int> &ids) = 0;
};
#endif
