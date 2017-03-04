#include <pattern_follower/robot_control.h>

RobotControl::RobotControl(const cv::Point2d &setPoint, const double &maxVel,
                           const double &acceleration, const double &threshLow,
                           const double &threshHigh, const double &densityB,
                           const double &robRadius, const int &mapSize,
                           const int &resolution, const double &safety,
                           const int &histRadius, const int &maxSize,
                           const int &alpha, const double &mu1,
                           const double &mu2, const double &mu3)
    : robot_(new Robot(setPoint, maxVel, robRadius * resolution / 100.0,
                       acceleration)),
      vfh_(new VFH(threshLow, threshHigh, robRadius, densityB, mapSize,
                   resolution, safety, histRadius, maxSize, alpha, mu1, mu2,
                   mu3)){};

void RobotControl::calculateRobotSpeeds(const std::vector<cv::Point2d> &points,
                                        const cv::Point2d &target) {}
