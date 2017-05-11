#include <pattern_follower/histogram.h>

Histogram::Histogram(const cv::FileNode &fn) {
  alpha_ = fn["alpha"];
  // alpha is chosen so bins is int
  bins_ = 360 / alpha_;
  densityB_ = fn["density_b"];
  double histogramRadius = fn["histogram_size"];
  // a - b*((r-1)/2) = 1
  densityA_ = (double)(1.0 + densityB_ * (histogramRadius - 1.0) *
                                 (histogramRadius - 1.0) / 4.0);
  binDensities_ = std::vector<int>(bins_, 1);
  magnitude_ = std::vector<double>(bins_, 0.0);
  threshLow_ = fn["threshold_low"];
  threshHigh_ = fn["threshold_high"];
  scanerAngle_ = (double)fn["scaner_angle"];
  max_ = (double)fn["robot_pos"] + (double)fn["histogram_size"];
  min_ = (double)fn["robot_pos"] - (double)fn["histogram_size"];
  maxAngle_ = 90 + scanerAngle_ / 2;
  minAngle_ = (90 - scanerAngle_ / 2);
  minAngle_ = minAngle_ < 0 ? minAngle_ + 360 : minAngle_;
}

void Histogram::update(const std::vector<std::vector<Map::Grid>> &grid) {
  calculateDensities(grid);
}

double Histogram::ratio(const int &candidate) {
  double h = std::min(magnitude_[candidate], threshHigh_);
  return 1.0 - h / threshHigh_;
}

void Histogram::calculateDensities(
    const std::vector<std::vector<Map::Grid>> &grid) {

  for (int bin = 0; bin < bins_; bin++) {
    int angle = bin * alpha_;
    // if angle outside lase scaner angle, ignore this histogram sector.
    if ((scanerAngle_ <= 180 && (angle <= minAngle_ || angle >= maxAngle_)) ||
        (scanerAngle_ > 180 && (angle <= minAngle_ && angle >= maxAngle_)))
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
