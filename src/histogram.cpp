#include <pattern_follower/histogram.h>

Histogram::Histogram(const double &threshLow, const double &threshHigh,
                     const double &densityB, const int &robotPos,
                     const int &alpha, const int &histRadius) {
  alpha_ = alpha;
  // alpha is chosen so bins is int
  bins_ = 360 / alpha;
  densityB_ = densityB;
  // a - b*((r-1)/2) = 1
  // HOTFIX
  densityA_ = (double)(1.0 + densityB_ * (histRadius * 10.0 - 1.0) *
                                 (histRadius * 10.0 - 1.0) / 4.0);
  binDensities_ = std::vector<int>(bins_, 1);
  threshLow_ = threshLow;
  threshHigh_ = threshHigh;
  max_ = robotPos + histRadius;
  min_ = robotPos - histRadius;
};

void Histogram::update(const std::vector<std::vector<Map::Grid>> &grid) {
  calculateDensities(grid);
  std::cout << "zde\n";
  // binarize();
  for (size_t i = 0; i < binDensities_.size(); i++) {
    std::cout << binDensities_[i] << " ";
  }
  std::cout << "\n";
}

void Histogram::calculateDensities(
    const std::vector<std::vector<Map::Grid>> &grid) {
  for (int bin = 0; bin < bins_ / 2; bin++) {
    int angle = bin * alpha_;
    double magnitude = 0;
    for (int i = min_; i <= max_; i++) {
      for (int j = min_; j <= max_; j++) {
        if (angle >= grid[i][j].beta - grid[i][j].gamma &&
            angle <= grid[i][j].beta + grid[i][j].gamma) {
          double tmp_mag = (double)(grid[i][j].cost * grid[i][j].cost);
          tmp_mag *= tmp_mag *= (densityA_ - densityB_ * grid[i][j].distance *
                                                 grid[i][j].distance);
          magnitude += tmp_mag;
        }
      }
    }
    std::cout << magnitude << " ";
    binarize(bin, magnitude);
  }
  std::cout << "\n";
}

void Histogram::binarize(const int &bin, const double &mag) {
  // for (int bin = 0; bin < bins_; bin++) {
  if (mag > threshHigh_)
    binDensities_[bin] = 1;
  else if (mag < threshLow_)
    binDensities_[bin] = 0;
}
