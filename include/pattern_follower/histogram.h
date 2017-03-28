#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <iostream>
#include <pattern_follower/map.h>
#include <pattern_follower/utils.h>
#include <vector>

#define THRESHOLD_LOW 250.0
#define THRESHOLD_HIGH 350.0
#define SCANER_ANGLE 220

class Histogram {

public:
  Histogram(const double &densityB, const int &robotPos, const int &alpha = 5,
            const int &histRadius = 10);

  void update(const std::vector<std::vector<Map::Grid>> &grid);

  const std::vector<int> getDensities() const { return binDensities_; }
  double ratio(const int &candidate);

private:
  int alpha_, bins_, max_, min_, maxAngle_, minAngle_;
  double densityA_, densityB_, threshLow_, threshHigh_, robRadAndSafety_;
  std::vector<int> binDensities_;
  std::vector<double> magnitude_;

  void calculateDensities(const std::vector<std::vector<Map::Grid>> &grid);
  void binarize(const int &bin);
};
#endif
