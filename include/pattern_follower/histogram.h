#ifndef MAP_H
#define MAP_h

#include <pattern_follower/map.h>
#include <vector>
class Histogram {

public:
  Histogram(const int &_alpha, const double &_threshold,
            const double &_density_a, const double &_density_b) {
    alpha = _alpha;
    bins = 360 / alpha;
    threshold = _threshold;
    density_a = _density_a;
    density_b = _density_b;
    densities = std::vector<int>(bins);
  };

  void update_histogram(const Map &map);
  void smooth_histogram(const int &window);

private:
  int alpha, bins;
  double threshold, density_a, density_b;
  std::vector<int> densities;
};
#endif
