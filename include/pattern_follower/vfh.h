#ifndef VFH_H
#define VFH_h

#include <cmath>
#include <pattern_follower/utils.h>
#include <vector>
class VFH {
public:
  VFH(const int &_maxSize, const int &_alpha, const double &_threshold)
      : maxSize(_maxSize), alpha(_alpha), threshold(_threshold){};

  double avoidObstacle(const std::vector<int> &bins, const double &dist);

private:
  int maxSize, alpha;
  double threshold;
  std::vector<std::vector<int>> candidateValleys;

  void findCandidates(const std::vector<int> &bins);
  int closestCandidate(const double &dist);
  double steeringDirection(const double &dist, const int &candidate);
};
#endif
