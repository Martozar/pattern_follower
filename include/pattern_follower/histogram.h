/**
 * @file histogram.h
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <iostream>
#include <pattern_follower/map.h>
#include <pattern_follower/utils.h>
#include <vector>

class Histogram {

public:
  Histogram(const cv::FileNode &fn);

  const std::vector<int> getDensities() const { return binDensities_; }

  /**
   * Update histogram based on new grid map.
   * @param [in] grid local map.
   */
  void update(const std::vector<std::vector<Map::Grid>> &grid);

  /**
   * Calculates speed ratio for choosed sector of histogram.
   * @param candidate choosed sector of histogram
   * @return ratio.
   */
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
