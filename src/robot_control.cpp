#include <pattern_follower/robot_control.h>

RobotControl::RobotControl(const cv::FileNode &fn, const bool &simulation)
    : robot_(new Robot(fn["Robot"], simulation)), vfh_(new VFH(fn["VFH"])),
      kf_(new KalmanFilter_(fn["KalmanFilter"])) {
  simulation_ = simulation;
}

void RobotControl::calculateRobotSpeeds(const std::vector<cv::Point2d> &points,
                                        const cv::Point2d &target,
                                        const bool &suceed) {
  cv::Point2d targetForVfh = target;
  kf_->prediction();
  if (suceed) {
    // Convert (d, phi) to (x, y)
    cv::Point2d targetPos = calculateTargetPosition(target);
    kf_->update(targetPos.x, targetPos.y);
  } else {
    // Convert (x, y) to (d, phi)
    targetForVfh = calculateTargetDistance(kf_->getX(), kf_->getY());
  }

  double ratio{1.0};
  // Choose new robot direction.
  double dir = vfh_->avoidObstacle(points, targetForVfh, robot_->getH(), ratio);
  // robot_->setMaxVel(ratio);

  // Apply new robot direction and
  robot_->move(-dir, targetForVfh.x);
}

cv::Point2d
RobotControl::calculateTargetPosition(const cv::Point2d &targetDist) {
  cv::Point2d targetPos;

  double fi = targetDist.y + robot_->getH();
  double xt = targetDist.x * std::cos(fi) / std::cos(targetDist.y);
  double yt = targetDist.x * std::sin(fi) / std::cos(targetDist.y);

  targetPos.x = xt + robot_->getX();
  targetPos.y = yt + robot_->getY();

  return targetPos;
}

cv::Point2d RobotControl::calculateTargetDistance(const double &x,
                                                  const double &y) {
  cv::Point2d targetDist;

  double xr = robot_->getX();
  double yr = robot_->getY();
  double fi = std::atan2(y - yr, x - xr);
  double a = std::sqrt((y - yr) * (y - yr) + (x - xr) * (x - xr));

  targetDist.y = fi - robot_->getH();
  targetDist.x = a * std::cos(targetDist.y);

  return targetDist;
}
