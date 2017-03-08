#include <pattern_follower/map.h>

Map::Map(const int &size, const int &resolution, const int &robotRad,
         const double &safety) {
  size_ = size;
  resolution_ = resolution;
  robotPos_ = size_ / 2;
  robotRadAndSafe_ = (((double)robotRad / 2.0) + safety) * (double)resolution;
};

void Map::init() {
  for (int i = 0; i < size_; i++) {
    std::vector<Map::Grid> tmp;
    for (int j = 0; j < size_; j++) {
      Grid g;
      g.cost = costs::FREE;
      double x = (robotPos_ - i);
      double y = (robotPos_ - j);
      g.location = cv::Point2i(x * resolution_, y * resolution_);
      g.beta = atanInPosDeg(x, y);
      g.distance = std::sqrt(x * x + y * y) * resolution_;
      if (g.distance > 0)
        g.gamma = radToDeg(std::asin(robotRadAndSafe_ / g.distance));
      else
        g.gamma = 0.0;
      tmp.push_back(g);
    }
    map_.push_back(tmp);
  }
}

void Map::update(const std::vector<cv::Point2d> &points,
                 const cv::Point2d &target) {
  for (int i = 0; i < size_; i++) {
    for (int j = 0; j < size_; j++) {
      map_[i][j].cost = costs::FREE;
    }
  }
  // map_[robotPos_][robotPos_].cost = costs::ROBOT;
  for (int i = 0; i < points.size(); i++) {
    int x = robotPos_ - std::round(points[i].x / (double)resolution_);
    int y = robotPos_ - std::round(-points[i].y / (double)resolution_);
    map_[x][y].cost++;
  }

  /*int x = robotPos_ - std::round(target.x * 100 / (double)resolution_);
  int y = robotPos_ -
          std::round(target.x * 100 * std::tan(target.y) / (double)resolution_);
  if (x >= 0 && y >= 0)
    map_[x][y].cost = 0;*/
  // std::cout << map_.size() << "\n";
}

void Map::show() {
  int res = resolution_ * 2;
  cv::Mat canvas = cv::Mat::zeros((size_ + 2) * res, (size_ + 2) * res, CV_32F);
  cvtColor(canvas, canvas, CV_GRAY2BGR);
  for (int i = 0; i < size_; i++) {
    for (int j = 0; j < size_; j++) {
      double x = res * (i + 1);
      double y = res * (j + 1);
      cv::putText(canvas, std::to_string((int)map_[i][j].cost),
                  cv::Point(x + res / 2, y + res / 2),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
                  cv::Scalar(255, 255, 255));
      cv::rectangle(canvas, cv::Rect(x, y, res, res), cv::Scalar(255, 255, 255),
                    1);
    }
  }
  cv::putText(canvas, std::to_string(costs::ROBOT),
              cv::Point(robotPos_ * res + res / 2, robotPos_ * res + res / 2),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255, 255, 255));
  imshow("map", canvas);
}
