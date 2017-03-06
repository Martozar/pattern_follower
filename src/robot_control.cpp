#include <pattern_follower/robot_control.h>

RobotControl::RobotControl(const cv::Point2d &setPoint, const double &maxVel,
                           const double &acceleration, const double &threshLow,
                           const double &threshHigh, const double &densityB,
                           const double &robRadius, const int &mapSize,
                           const int &resolution, const double &safety,
                           const int &histRadius, const int &maxSize,
                           const int &alpha, const double &mu1,
                           const double &mu2, const double &mu3,
                           bool simulation)
    : robot_(new Robot(setPoint, maxVel, robRadius * resolution / 200.0,
                       acceleration)),
      vfh_(new VFH(mapSize, resolution, robRadius, safety, threshLow,
                   threshHigh, densityB, histRadius, alpha, maxSize, mu1, mu2,
                   mu3)) {
  simulation_ = simulation;
};

void RobotControl::calculateRobotSpeeds(const std::vector<cv::Point2d> &points,
                                        const cv::Point2d &target) {
  double count = 0.0;
  double dir = vfh_->avoidObstacle(points, target, robot_->getH());
  std::cout << dir << "\n";

  robot_->move(dir, target.x, simulation_);
  while (count < 1e5)
    count++;
}
