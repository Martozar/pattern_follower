#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <iostream>
#include <pattern_follower/map.h>
#include <pattern_follower/utils.h>
#include <vector>

class Histogram {

public:
  Histogram(const cv::FileNode &fn);

  void update(const std::vector<std::vector<Map::Grid>> &grid);

  const std::vector<int> getDensities() const { return binDensities_; }
  double ratio(const int &candidate);

private:
  int alpha_, bins_, max_, min_, maxAngle_, minAngle_;
  double densityA_, densityB_, threshLow_, threshHigh_, scanerAngle_;
  std::vector<int> binDensities_;
  std::vector<double> magnitude_;

  void calculateDensities(const std::vector<std::vector<Map::Grid>> &grid);
  void binarize(const int &bin);
};
#endif
