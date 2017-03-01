#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <iostream>
#include <pattern_follower/map.h>
#include <pattern_follower/utils.h>
#include <vector>
class Histogram {

public:
  Histogram(const int &alpha, const int &radius, const double &density_b) {
    alpha_ = alpha;
    radius_ = radius;
    bins_ = 360 / alpha;
    densityB_ = density_b;
    densityA_ = densityB_ * std::sqrt(2) * radius;
    densities_ = std::vector<int>(bins_);
  };

  void update_histogram(const std::vector<std::vector<Map::Grid>> &grid);
  void smooth_histogram(const int &window);
  const std::vector<int> getDensities() const { return densities_; };

private:
  int alpha_, bins_, radius_;
  double densityA_, densityB_;
  std::vector<int> densities_;
};
#endif
