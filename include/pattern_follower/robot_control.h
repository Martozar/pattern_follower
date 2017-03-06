#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <pattern_follower/robot.h>
#include <pattern_follower/vfh.h>

class RobotControl {
public:
  RobotControl(const cv::Point2d &setPoint, const double &maxVel,
               const double &acceleration, const double &threshLow,
               const double &threshHigh, const double &densityB,
               const double &robRadius = 1.0, const int &mapSize = 21,
               const int &resolution = 20, const double &safety = 0.0,
               const int &histRadius = 10, const int &maxSize = 16,
               const int &alpha = 5, const double &mu1 = 5.0,
               const double &mu2 = 2.0, const double &mu3 = 2.0,
               bool simulation = true);
  void calculateRobotSpeeds(const std::vector<cv::Point2d> &points,
                            const cv::Point2d &target);

  const std::unique_ptr<Robot> &getRobot() const { return robot_; }

private:
  std::unique_ptr<Robot> robot_;
  std::unique_ptr<VFH> vfh_;
  bool simulation_;
};
#endif
