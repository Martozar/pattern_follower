#include <pattern_follower/robot_control.h>

RobotControl::RobotControl(const cv::FileNode &fn, const bool &simulation)
    : robot_(new Robot(fn["Robot"], simulation)), vfh_(new VFH(fn["VFH"])) {
  simulation_ = simulation;
}

void RobotControl::calculateRobotSpeeds(const std::vector<cv::Point2d> &points,
                                        const cv::Point2d &target) {

  double ratio;
  double dir = vfh_->avoidObstacle(points, target, robot_->getH(), ratio);
  robot_->setMaxVel(ratio);
  std::cout << robot_->getMaxVel() << "\n";
  robot_->move(dir, target.x);
}
