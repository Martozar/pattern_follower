#include <pattern_follower/histogram.h>

Histogram(const cv::FileNode &fn) {
  alpha_ = fn["alpha"];
  // alpha is chosen so bins is int
  bins_ = 360 / alpha;
  densityB_ = fn["density_b"];
  // a - b*((r-1)/2) = 1
  // HOTFIX
  densityA_ =
      (double)(1.0 + densityB_ * (histRadius - 1.0) * (histRadius - 1.0) / 4.0);
  binDensities_ = std::vector<int>(bins_, 1);
  magnitude_ = std::vector<double>(bins_, 0.0);
  threshLow_ = fn["threshold_low"];
  threshHigh_ = fn["threshold_high"];
  max_ = fn["robot_pos"] + fn["histogram_size"];
  min_ = fn["robot_pos"] - fn["histogram_size"];
  maxAngle_ = 90 + fn["scaner_angle"] / 2;
  minAngle_ = (90 - fn["scaner_angle"] / 2);
  minAngle_ = minAngle_ < 0 ? minAngle_ + 360 : minAngle_;
}

void Histogram::update(const std::vector<std::vector<Map::Grid>> &grid) {
  calculateDensities(grid);
}

void Histogram::calculateDensities(
    const std::vector<std::vector<Map::Grid>> &grid) {
  for (int bin = 0; bin < bins_; bin++) {
    int angle = bin * alpha_;
    if ((SCANER_ANGLE <= 180 && (angle <= minAngle_ || angle >= maxAngle_)) ||
        (SCANER_ANGLE > 180 && (angle <= minAngle_ && angle >= maxAngle_)))
      continue;
    double magnitude = 0;
    for (int i = min_; i <= max_; i++) {
      for (int j = min_; j <= max_; j++) {
        if (angle >= grid[i][j].beta - grid[i][j].gamma &&
            angle <= grid[i][j].beta + grid[i][j].gamma) {
          double tmp_mag = (double)(grid[i][j].cost * grid[i][j].cost);
          tmp_mag *= (densityA_ -
                      densityB_ * grid[i][j].distance * grid[i][j].distance);
          magnitude_[bin] += tmp_mag;
        }
      }
    }

    binarize(bin);
  }
}

void Histogram::binarize(const int &bin) {
  if (magnitude_[bin] > threshHigh_)
    binDensities_[bin] = 1;
  else if (magnitude_[bin] < threshLow_)
    binDensities_[bin] = 0;
  magnitude_[bin] = 0.0;
}

double Histogram::ratio(const int &candidate) {
  double h = std::min(magnitude_[h], threshHigh_);
  return 1.0 - h / threshHigh_;
}
