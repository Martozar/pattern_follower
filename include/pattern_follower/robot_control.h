/**
 * @file robot_control.h
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <pattern_follower/kalman_filter.h>
#include <pattern_follower/robot.h>
#include <pattern_follower/vfh.h>

class RobotControl {
public:
  RobotControl(const cv::FileNode &fn, const bool &simulation = true);

  std::unique_ptr<Robot> &getRobot() { return robot_; }

  /**
   * Calculates robot speeds and apply them to it.
   *
   * @param [in] points data from rangefinder. WARNING: must have the units as
   * map resolution and be in (x,y) format.
   * @param [in] target position against camera. WARNING: must be in (d, phi)
   * format, where d is distance from camera to marker and has the same units as
   * map resolution, and phi is angle between camera and marker.
   * @param [in] suceed marker was successfully detekted. If TRUE KF is updated,
   * else only prediction step is used.
   */
  void calculateRobotSpeeds(const std::vector<cv::Point2d> &points,
                            const cv::Point2d &target, const bool &suceed);

private:
  std::unique_ptr<Robot> robot_;
  std::unique_ptr<VFH> vfh_;
  std::unique_ptr<KalmanFilter_> kf_;
  cv::Point2d calculateTargetPosition(const cv::Point2d &target);
  cv::Point2d calculateTargetDistance(const double &dist, const double &angle);
  bool simulation_;
};
#endif
