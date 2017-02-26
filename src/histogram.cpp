#include <pattern_follower/histogram.h>

void Histogram::update_histogram(const Map &map) {
  auto grid = map.getMap();
  int robot_pos = map.getRobotPos();
  for (int i = 0; i < grid.size(); i++) {
    for (int j = 0; j < grid.size(); j++) {
      double angle = atan2((double)(j - robot_pos), (double)(i - robot_pos));
      double density = grid[i][j].cost * grid[i][j].cost;
      density *= (density_a -
                  density_b * sqrt((i - robot_pos) * (i - robot_pos) +
                                   (j - robot_pos) * (j - robot_pos)));
      densities[(int)floor(angle / alpha)] += density;
    }
  }
}

void Histogram::smooth_histogram(const int &window) {
  int winMidSize = window / 2;
  for (int i = winMidSize / 2; i < bins - winMidSize; i++) {
    float mean = 0;
    for (int j = i - winMidSize; j <= (i + winMidSize); j++) {
      mean += densities[j];
    }

    densities[i] = mean / window;
  }
}
