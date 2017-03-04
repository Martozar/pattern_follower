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

  Map(const int &size = 21, const int &resolution = 20);

  void init();
  void update(const std::vector<cv::Point2d> &points,
              const cv::Point2d &target);
  void show();
  const std::vector<std::vector<Map::Grid>> &getMap() const { return map_; }
  const int getRobotPos() const { return robotPos_; }

private:
  enum costs {
    ROBOT = 10,
    FREE = 0,
    DANGEROUS = 2,
    GOAL = 10,
    OBSTACLE = 10000
  };
  int size_, resolution_, robotPos_;
  std::vector<std::vector<Map::Grid>> map_;
};

#endif
