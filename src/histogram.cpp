#include <pattern_follower/histogram.h>

void Histogram::update_histogram(
    const std::vector<std::vector<Map::Grid>> &grid) {
  // auto grid = map.getMap();
  densities_ = std::vector<int>(bins_);
  int robot_pos = grid.size() / 2;
  int min = robot_pos - radius_;
  int max = robot_pos + radius_;
  for (int i = min; i <= max; i++) {
    for (int j = min; j <= max; j++) {
      double angle = atan2((double)(j - robot_pos), (double)(i - robot_pos));
      angle = radToDeg(angle, true);
      double density = grid[i][j].cost * grid[i][j].cost;
      density *=
          (densityA_ - densityB_ * sqrt((i - robot_pos) * (i - robot_pos) +
                                        (j - robot_pos) * (j - robot_pos)));
      densities_[(int)floor(angle / alpha_)] += density;
    }
  }
  std::cout << "Before smooth\n";
  for (int i = 0; i < densities_.size(); i++) {
    std::cout << densities_[i] << " ";
  }
  std::cout << "\n";
}

void Histogram::smooth_histogram(const int &window) {
  int winMidSize = window / 2;
  for (int i = winMidSize / 2; i < bins_ - winMidSize; i++) {
    float mean = 0;
    for (int j = i - winMidSize; j <= (i + winMidSize); j++) {
      mean += densities_[j];
    }

    densities_[i] = mean / window;
  }
  std::cout << "After smooth\n";
  for (int i = 0; i < densities_.size(); i++) {
    std::cout << densities_[i] << " ";
  }
  std::cout << "\n";
}
