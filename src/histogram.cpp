#include <pattern_follower/histogram.h>

Histogram::Histogram(const double &threshLow, const double &threshHigh,
                     const double &robRadius, const int &robotPos,
                     const double &densityB, const double &safety,
                     const int &alpha, const int &histRadius) {
  alpha_ = alpha;
  histRadius_ = histRadius;
  robotPos_ = robotPos;
  // alpha is chosen so bins is int
  bins_ = 360 / alpha;
  densityB_ = densityB;
  // a - b*((r-1)/2) = 1
  densityA_ = (double)(1.0 + densityB_ * (histRadius_ - 1.0) *
                                 (histRadius_ - 1.0) / 4.0);
  densities_ = std::vector<double>(bins_);
  binDensities_ = std::vector<int>(bins_);
  lastDensities_ = binDensities_;
  threshLow_ = threshLow;
  threshHigh_ = threshHigh;
  robRadAndSafety_ = robRadius + safety;
  max_ = robotPos_ + histRadius_;
  min_ = robotPos_ - histRadius_;
};

void Histogram::update(const std::vector<std::vector<Map::Grid>> &grid) {
  calculateDensities(grid);
  std::cout << "zde\n";
  // binarize();
  for (size_t i = 0; i < densities_.size(); i++) {
    std::cout << binDensities_[i] << " ";
  }
  std::cout << "\n";
}
void Histogram::calculateDensities(
    const std::vector<std::vector<Map::Grid>> &grid) {
  densities_ = std::vector<double>(bins_);
  std::cout << robotPos_ << "pica\n";
  double magnitude, dist, beta, gamma;
  gamma = 0;
  for (int bin = 0; bin < bins_; bin++) {
    int angle = bin * alpha_;
    angle -= angle > 180 ? 360 : 0;
    for (int i = min_; i <= max_; i++) {
      for (int j = min_; j <= max_; j++) {
        dist = (double)((i - robotPos_) * (i - robotPos_));
        dist += (double)((j - robotPos_) * (j - robotPos_));
        dist = std::sqrt(dist);
        beta = radToDeg(
            std::atan2((double)(robotPos_ - j), (double)(robotPos_ - i)));
        gamma = radToDeg(std::asin(robRadAndSafety_ / dist));
        //
        if (angle >= beta - gamma && angle <= beta + gamma) {
          magnitude = (double)(grid[i][j].cost * grid[i][j].cost);
          magnitude *= (densityA_ - densityB_ * dist * dist);
          densities_[bin] += magnitude;
        }
      }
    }

    std::cout << densities_[bin] << " ";
    binarize(bin);
  }
  std::cout << "\n";
  std::cout << threshLow_ << " " << threshHigh_ << "\n";
}

void Histogram::binarize(int &bin) {
  // for (int bin = 0; bin < bins_; bin++) {
  if (densities_[bin] > threshHigh_)
    binDensities_[bin] = 1;
  else if (densities_[bin] < threshLow_)
    binDensities_[bin] = 0;
  else {
    binDensities_[bin] = lastDensities_[bin]; // lastDensities_[bin];
  }
  //}
  lastDensities_[bin] = binDensities_[bin]; /*
   for (size_t i = 0; i < densities_.size(); i++) {
     std::cout << densities_[i] << " ";
   }*/
}
