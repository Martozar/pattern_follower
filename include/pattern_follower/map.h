/**
 * @file map.h
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

#ifndef MAP_H
#define MAP_H

#include <opencv2/opencv.hpp>
#include <pattern_follower/utils.h>

#define TARGET_RADIUS 2
class Map {

public:
  struct Grid {
    cv::Point2i location;
    double cost;
    double distance;
    double beta;
    double gamma;
  };

  Map(const cv::FileNode &fn);

  const std::vector<std::vector<Map::Grid>> &getMap() const { return map_; }

  const int getRobotPos() const { return robotPos_; }

  /**
   * Initialize local map with particular size and set distances and angles from
   * center to every cell in the map.
   */
  void init();

  /**
   * Update local map with rangefinder data.
   *
   * @param [in] points data from rangefinder. WARNING: must have the units as
   * map reoslution and be in (x,y) format.
   * @param [in] target position against camera. WARNING: must be in (d, phi)
   * format, where d is distance from camera to marker and has the same units as
   * map resolution, and phi is angle between camera and marker.
   */
  void update(const std::vector<cv::Point2d> &points,
              const cv::Point2d &target);

  /**
   *  Draw current map to canvas.
   */
  void show();

  /**
   * Draw circle with targetRadius_ radius around target center point.
   * @param [in] target position against camera. WARNING: must be in (d, phi)
   * format, where d is distance from camera to marker and has the same units as
   */
  void drawCircle(const cv::Point2d &target);

private:
  enum costs {
    ROBOT = 10,
    FREE = 0,
    DANGEROUS = 2,
    GOAL = 10,
    OBSTACLE = 10000
  };
  int size_, robotPos_, targetRadius_, showMap_;
  double resolution_, robotRadAndSafe_;
  std::vector<std::vector<Map::Grid>> map_;
};

#endif
