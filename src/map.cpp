
#include <pattern_follower/map.h>

void Map::init() {
  for (int i = 0; i < size; i++) {
    std::vector<Grid> tmp;
    for (int j = 0; j < size; j++) {
      Grid g;
      g.cost = costs::FREE;
      g.location = cv::Point2i((robot_pos - i) * resolution,
                               (robot_pos - j) * resolution);
      std::cout << g.location << std::endl;
      tmp.push_back(g);
    }
    map.push_back(tmp);
  }
  map[robot_pos][robot_pos].cost = costs::ROBOT;
}

void Map::update(const std::vector<cv::Point2d> &points) {
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      map[i][j].cost = costs::FREE;
    }
  }
  map[robot_pos][robot_pos].cost = costs::ROBOT;
  for (int i = 0; i < points.size(); i++) {
    int x = robot_pos - std::round(points[i].x / (double)resolution);
    int y = robot_pos - std::round(points[i].y / (double)resolution);
    map[x][y].cost++; /*
     for (int x_n = x - 1; x_n <= x + 1; x_n++) {
       for (int y_n = y - 1; y_n <= y + 1; y_n++) {
         if (x_n >= 0 && y_n >= 0 && x_n < size && y_n < size &&
             map[x_n][y_n].cost == costs::FREE)
           map[x_n][y_n].cost = costs::DANGEROUS;
       }
     }*/
  }
}

void Map::show() {
  cv::Mat canvas =
      cv::Mat::zeros((size + 2) * resolution, (size + 2) * resolution, CV_32F);
  cvtColor(canvas, canvas, CV_GRAY2BGR);
  for (unsigned int i = 0; i < size; i++) {
    for (unsigned int j = 0; j < size; j++) {
      double x = resolution * (i + 1);
      double y = resolution * (j + 1);
      /*cv::Scalar colour;
      switch ((int)map[i][j].cost) {
      case costs::ROBOT:
        colour = cv::Scalar(255, 0, 0);
        break;
      case costs::DANGEROUS:
        colour = cv::Scalar(0, 255, 255);
        break;
      case costs::GOAL:
        colour = cv::Scalar(0, 255, 0);
        break;
      case costs::OBSTACLE:
        colour = cv::Scalar(0, 0, 0);
        break;
      default:

        colour = cv::Scalar(255, 255, 255);
        break;
      }
      cv::rectangle(canvas, cv::Rect(x, y, resolution, resolution), colour,
      -1);*/
      /*cv::putText(canvas, std::to_string(map[i][j].cost), cv::Point(x, y),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
                  cv::Scalar(255, 255, 255));*/
      cv::rectangle(canvas, cv::Rect(x, y, resolution, resolution),
                    cv::Scalar(255, 255, 255), 1);
    }
  }
  imshow("map", canvas);
  // char q = cv::waitKey(1);
}
