#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <iostream>
#include <pattern_follower/map.h>
#include <pattern_follower/utils.h>
#include <vector>
class Histogram {

public:
  Histogram(const double &threshLow, const double &threshHigh,
            const double &robRadius, const int &robotPos,
            const double &densityB, const double &safety = 0.0,
            const int &alpha = 5, const int &histRadius = 10);

  void update(const std::vector<std::vector<Map::Grid>> &grid);
  const std::vector<int> getDensities() const { return densities_; }

private:
  int alpha_, bins_, histRadius_, robotPos_, max_, min_;
  double densityA_, densityB_, threshLow_, threshHigh_, robRadAndSafety_;
  std::vector<int> densities_, lastDensities_;

  void calculateDensities(const std::vector<std::vector<Map::Grid>> &grid);
  void binarize();
};
#endif
