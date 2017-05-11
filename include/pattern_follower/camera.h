/**
 * @file camera.h
 * @breaf Read frame from camera, find markers and calculate distances
 * from camera to them.
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <pattern_follower/arucodetector.h>
#include <pattern_follower/cam_calibration.h>
#include <pattern_follower/camera_calibration.h>
#include <pattern_follower/contourfinder.h>
#include <pattern_follower/kalman_filter.h>
#include <pattern_follower/measurement.h>
#include <pattern_follower/roidetector.h>
#include <pattern_follower/templatematcher.h>

class Camera {
public:
  Camera(const FileNode &fn);

  /**
   * Reads frame from camera and call {@link #proceed proceed}.
   * @see #proceed
   * @param [out] angle calculated angle between marker and camera
   * @param [out] distance calculated angle from marker to camera
   * @retval TRUE if marker was detected
   * @retval FALSE if marker was not detected
   */
  bool proceed(double &angle, double &distance);

  /**
   * Detects marker and calculates distance to it.
   * @see #proceed
   * @param [in] image camera frame
   * @param [out] angle calculated angle between marker and camera
   * @param [out] distance calculated angle from marker to camera
   * @retval TRUE if marker was detected
   * @retval FALSE if marker was not detected
   */
  bool proceed(cv::Mat &image, double &angle, double &distance);

private:
  std::unique_ptr<Measurement> measurement_;
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<KalmanFilter_> kalmanFilter_;
  cv::VideoCapture cap_;
};
#endif
