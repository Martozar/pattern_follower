#ifndef MAP_H
#define MAP_H

#include <cmath>
#include <opencv2/opencv.hpp>

class Map {

public:
  struct Grid {
    cv::Point2i location;
    double cost;
  };

  Map(const int &_size, const int &_resolution);

  void init();
  void update(const std::vector<cv::Point2d> &points,
              const cv::Point2d &target);
  void show();
  const std::vector<std::vector<Map::Grid>> getMap() const { return map; }
  const int getRobotPos() const { return robot_pos; }

private:
  enum costs {
    ROBOT = 0,
    FREE = 0,
    DANGEROUS = 2,
    GOAL = 10,
    OBSTACLE = 10000
  };
  int size, resolution, robot_pos;
  std::vector<std::vector<Map::Grid>> map;
};

#endif
