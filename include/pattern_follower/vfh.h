#ifndef VFH_H
#define VFH_h

#include <cmath>
#include <pattern_follower/histogram.h>
#include <pattern_follower/map.h>
#include <pattern_follower/utils.h>
#include <vector>

#define MAP_SIZE 81
#define RESOLUTION 5
#define HISTOGRAM_SIZE 40
#define ROBOT_RADIUS 2.0
#define ALPHA 2
#define MAX_SIZE 16
#define MU_1 5
#define MU_2 2
#define MU_3 2

class VFH {
public:
  VFH(const FileNode &fn);

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
