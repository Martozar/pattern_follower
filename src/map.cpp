#include <pattern_follower/map.h>

Map::Map(const cv::FileNode &fn) {
  size_ = fn["map_size"];
  resolution_ = fn["resolution"];
  robotPos_ = size_ / 2;
  robotRadAndSafe_ = (double)fn["cell_robot_radius"] + (double)fn["safety"];
  targetRadius_ = fn["target_radius"];
}

void Map::init() {
  for (int i = 0; i < size_; i++) {
    std::vector<Map::Grid> tmp;
    for (int j = 0; j < size_; j++) {
      Grid g;
      g.cost = costs::FREE;
      double x = (robotPos_ - i);
      double y = (robotPos_ - j);
      g.location = cv::Point2i(x * resolution_, y * resolution_);

      g.distance = std::sqrt(x * x + y * y);
      if (g.distance > 0) {
        g.gamma = radToDeg(std::asin(robotRadAndSafe_ / g.distance));
        g.beta = atanInPosDeg(x, y);
      } else {
        g.gamma = 0.0;
        g.beta = -1;
      }
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
  for (int i = 0; i < points.size(); i++) {
    int x = robotPos_ - std::round(points[i].x / (double)resolution_);
    int y = robotPos_ - std::round(points[i].y / (double)resolution_);
    if (x >= 0 && x < size_ && y >= 0 && y < size_)
      map_[x][y].cost++;
  }
  drawCircle(target);
  // std::cout << "Rob " << robotRadAndSafe_ << "\n";
}

void Map::drawCircle(const cv::Point2d &target) {
  int x = robotPos_ - std::round(target.x * 100.0 / (double)resolution_);
  int y = robotPos_ - std::round(-target.x * 100.0 * std::tan(target.y) /
                                 (double)resolution_);
  for (int i = std::max(x - targetRadius_, 0);
       i <= std::min(x + targetRadius_, size_); i++) {
    for (int j = std::max(y - targetRadius_, 0);
         j < std::min(y + targetRadius_, size_); j++) {
      if ((x - i) * (x - i) + (y - j) * (y - j) <=
          targetRadius_ * targetRadius_)
        map_[i][j].cost = costs::FREE;
    }
  }
  // show();
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
