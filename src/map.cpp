
#include <pattern_follower/map.h>

Map::Map(const int &size, const int &resolution)
    : size_(size), resolution_(resolution), robotPos_(size_ / 2){};

void Map::init() {
  for (int i = 0; i < size_; i++) {
    std::vector<Map::Grid> tmp;
    for (int j = 0; j < size_; j++) {
      Grid g;
      g.cost = costs::FREE;
      g.location = cv::Point2i((robotPos_ - i) * resolution_,
                               (robotPos_ - j) * resolution_);
      tmp.push_back(g);
    }
    map_.push_back(tmp);
  }
  map_[robotPos_][robotPos_].cost = costs::ROBOT;
}

void Map::update(const std::vector<cv::Point2d> &points,
                 const cv::Point2d &target) {
  for (int i = 0; i < size_; i++) {
    for (int j = 0; j < size_; j++) {
      map_[i][j].cost = costs::FREE;
    }
  }
  map_[robotPos_][robotPos_].cost = costs::ROBOT;
  for (int i = 0; i < points.size(); i++) {
    int x = robotPos_ - std::round(points[i].y / (double)resolution_);
    int y = robotPos_ - std::round(points[i].x / (double)resolution_);
    map_[x][y].cost++;
  }
  int x = robotPos_ -
          std::round(target.x * std::tan(target.y) / (double)resolution_);
  int y = robotPos_ - std::round(target.x / (double)resolution_);
  map_[x][y].cost = 0;
  std::cout << map_.size() << "\n";
}

void Map::show() {
  cv::Mat canvas = cv::Mat::zeros((size_ + 2) * resolution_,
                                  (size_ + 2) * resolution_, CV_32F);
  cvtColor(canvas, canvas, CV_GRAY2BGR);
  for (int i = 0; i < size_; i++) {
    for (int j = 0; j < size_; j++) {
      double x = resolution_ * (i + 1);
      double y = resolution_ * (j + 1);
      cv::putText(canvas, std::to_string((int)map_[i][j].cost),
                  cv::Point(x + resolution_ / 2, y + resolution_ / 2),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
                  cv::Scalar(255, 255, 255));
      cv::rectangle(canvas, cv::Rect(x, y, resolution_, resolution_),
                    cv::Scalar(255, 255, 255), 1);
    }
  }
  cv::putText(canvas, std::to_string(costs::ROBOT),
              cv::Point(robotPos_ * resolution_ + resolution_ / 2,
                        robotPos_ * resolution_ + resolution_ / 2),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255, 255, 255));
  imshow("map", canvas);
}
