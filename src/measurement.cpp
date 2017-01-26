#include <pattern_follower/measurement.h>

double Measurement::distance(const int &pattern_width_pix) const {
  return (pattern_width * focal_length) / ((double)pattern_width_pix);
}

double Measurement::angle(const std::vector<Point2f> &vertices) const {
  Moments mu = moments(vertices, false);
  Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

  return (mc.x - center) * anglePerPixel;
}

double Measurement::distance(const std::vector<Point2f> &vertices) const {

  Point2f d1 = vertices.at(1) - vertices.at(0);
  Point2f d2 = vertices.at(2) - vertices.at(1);
  Point2f d3 = vertices.at(3) - vertices.at(2);
  Point2f d4 = vertices.at(0) - vertices.at(3);

  int dis1 = sqrt(d1.x * d1.x + d1.y * d1.y);
  int dis2 = sqrt(d2.x * d2.x + d2.y * d2.y);
  int dis3 = sqrt(d3.x * d3.x + d3.y * d3.y);
  int dis4 = sqrt(d4.x * d4.x + d4.y * d4.y);

  return fmin(fmin(distance(dis1), distance(dis2)),
              fmin(distance(dis3), distance(dis4)));
}
