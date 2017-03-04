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
  densities_ = std::vector<int>(bins_);
  lastDensities_ = densities_;
  threshLow_ = threshLow;
  threshHigh_ = threshHigh_;
  robRadAndSafety_ = robRadius + safety;
  max_ = robotPos_ + histRadius_;
  min_ = robotPos_ - histRadius_;
};

void Histogram::update(const std::vector<std::vector<Map::Grid>> &grid) {
  calculateDensities(grid);
  std::cout << "zde\n";
  binarize();
  /*for (size_t i = 0; i < densities_.size(); i++) {
    std::cout << densities_[i] << " ";
  }
  std::cout << "\n";*/
}
void Histogram::calculateDensities(
    const std::vector<std::vector<Map::Grid>> &grid) {
  densities_ = std::vector<int>(bins_);
  int gridSize = grid.size();
  std::cout << gridSize << "\n";
  double magnitude, dist, beta, gamma;
  for (int bin = 0; bin < bins_; bin++) {
    double angle = bin * alpha_ - 180;
    for (int i = min_; i <= max_; i++) {
      for (int j = min_; j <= max_; j++) {
        dist = (i - robotPos_) * (i - robotPos_);
        dist += (j - robotPos_) * (j - robotPos_);
        dist = std::sqrt(dist);
        beta = std::atan2(robotPos_ - j, robotPos_ - i);
        gamma = radToDeg(std::asin(robRadAndSafety_ / dist));
        //
        if (angle >= beta - gamma && angle <= beta + gamma) {
          magnitude = grid[i][j].cost * grid[i][j].cost;
          magnitude *= (densityA_ - densityB_ * dist);
          densities_[bin] += magnitude;
          /*if (bin == 0)
            std::cout << angle << " " << beta << " " << gamma << " ahoj\n";*/
        }
      }
    }
  }
  /*;*/
}

void Histogram::binarize() {
  for (int bin = 0; bin < bins_; bin++) {
    if (densities_[bin] > threshHigh_)
      densities_[bin] = 1;
    else if (densities_[bin] < threshLow_)
      densities_[bin] = 0;
    else {
      densities_[bin] = lastDensities_[bin];
    }
  }
  lastDensities_ = densities_;
  for (size_t i = 0; i < densities_.size(); i++) {
    std::cout << densities_[i] << " ";
  }
  std::cout << "\n";
}
