#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <pattern_follower/robot.h>
#include <pattern_follower/vfh.h>

class RobotControl {
public:
  RobotControl(const cv::FileNode &fn, const bool &simulation = true);

  void calculateRobotSpeeds(const std::vector<cv::Point2d> &points,
                            const cv::Point2d &target);

  std::unique_ptr<Robot> &getRobot() { return robot_; }

private:
  std::unique_ptr<Robot> robot_;
  std::unique_ptr<VFH> vfh_;
  bool simulation_;
};
#endif
