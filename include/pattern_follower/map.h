#ifndef MAP_H
#define MAP_h

#include <cmath>
#include <opencv2/opencv.hpp>
class Map {

  struct Grid {
    cv::Point2i location;
    double cost;
  };

public:
  Map(const int &_size, const int &_resolution)
      : size(_size), resolution(_resolution), robot_pos(size / 2){};
  void init();
  void update(const std::vector<cv::Point2d> &points);
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
  std::vector<std::vector<Grid>> map;
};

#endif
