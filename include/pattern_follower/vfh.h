/**
 * @file vfh.h
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

#ifndef VFH_H
#define VFH_h

#include <cmath>
#include <memory>
#include <pattern_follower/histogram.h>
#include <pattern_follower/map.h>
#include <pattern_follower/utils.h>
#include <vector>

class VFH {
public:
  VFH(const cv::FileNode &fn);

  /**
   * Caclulate new heading direction to avoid obstacles.
   *
   * @param [in] points data from rangefinder. WARNING: must have the units as
   * map resolution and be in (x,y) format.
   * @param [in] target position against camera. WARNING: must be in (d,
   * phi) format, where d is distance from camera to marker and has the same
   * units as map resolution, and phi is angle between camera and marker.
   * @param [in] curHead current robot heading direction.
   * @param [out] speedRatio speed ratio to update max robot speed.
   */
  double avoidObstacle(const std::vector<cv::Point2d> &points,
                       const cv::Point2d &target, const double &curHead,
                       double &speedRatio);

private:
  int maxSize_, alpha_, bins_;
  double mu1_, mu2_, mu3_, prevOrient_{0.0};
  std::vector<std::vector<int>> candidateValleys_;

  std::unique_ptr<Map> map_;
  std::unique_ptr<Histogram> histogram_;

  void findCandidates(const std::vector<int> &bins);
  double steeringDirection(const double &dist, const int &candidate);
  double calculateCost(const int &candidate, const int &target,
                       const int &curHeads);
  double costFunction(const int &c1, const int &c2);
  int delta(const int &c1, const int &c2);
  double chooseCandidate(const double &dist, const double &curHead);
};
#endif
