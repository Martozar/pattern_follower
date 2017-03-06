#ifndef MAP_H
#define MAP_H

#include <opencv2/opencv.hpp>
#include <pattern_follower/utils.h>

class Map {

public:
  struct Grid {
    cv::Point2i location;
    double cost;
    double distance;
    double beta;
    double gamma;
  };

  Map(const int &size = 21, const int &resolution = 20, const int &robotRad = 1,
      const double &safety = 0.0);

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
  int size_, resolution_, robotPos_, robotRadAndSafe_;
  std::vector<std::vector<Map::Grid>> map_;
};

#endif
