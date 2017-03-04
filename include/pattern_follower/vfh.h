#ifndef VFH_H
#define VFH_h

#include <cmath>
#include <pattern_follower/histogram.h>
#include <pattern_follower/map.h>
#include <pattern_follower/utils.h>
#include <vector>

class VFH {
public:
  VFH(const double &threshLow, const double &threshHigh,
      const double &robRadius, const double &densityB, const int &mapSize = 21,
      const int &resolution = 20, const double &safety = 0.0,
      const int &histRadius = 10, const int &maxSize = 16, const int &alpha = 5,
      const double &mu1 = 5.0, const double &mu2 = 2.0,
      const double &mu3 = 2.0);

  double avoidObstacle(const std::vector<cv::Point2d> &points,
                       const cv::Point2d &target, const double &curHead);

private:
  int maxSize_, alpha_, prevOrient_;
  double mu1_, mu2_, mu3_;
  std::vector<std::vector<int>> candidateValleys_;

  std::unique_ptr<Map> map_;
  std::unique_ptr<Histogram> histogram_;

  void findCandidates(const std::vector<int> &bins);
  double steeringDirection(const double &dist, const int &candidate);
  double calculateCost(const int &candidate, const int &target,
                       const int &curHeads);
  double costFunction(const int &c1, const int &c2);
  int chooseCandidate(const double &dist, const double &curHead);
};
#endif
