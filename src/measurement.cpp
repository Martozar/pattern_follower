#include <pattern_follower/measurement.h>

Measurement::Measurement(const FileNode &fn) {
  int frameWidth = (int)fn["frame_width"];
  center_ = frameWidth / 2;
  patternWidth_ = (double)fn["pattern_width"];
  double distance = (double)fn["distance"];
  double patternWidthPix = (double)fn["patter_width_pix"];
  focalLength_ = patternWidthPix * distance / patternWidth_;
  double fovx = 2.0 * atan(frameWidth / (2.0 * focalLength_));
  anglePerPixel_ = fovx / frameWidth;
}

Measurement::Measurement(const int &frameWidth, const double &patternWidth,
                         const double &distance, const double &patternWidthPix,
                         const double &fovx) {
  center_ = frameWidth / 2;
  patternWidth_ = patternWidth;
  focalLength_ =
      ((double)patternWidthPix * (double)distance) / ((double)patternWidth_);
  anglePerPixel_ = fovx / frameWidth;
}

double Measurement::distance(const double &patternWidthPix) const {
  return (patternWidth_ * focalLength_) / (patternWidthPix);
}

double Measurement::angle(const std::vector<Point2f> &vertices) const {
  // Find center of mass of detected marker.
  Moments mu = moments(vertices, false);
  Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

  return (center_ - mc.x) * anglePerPixel_;
}

double Measurement::distance(const std::vector<Point2f> &vertices) const {

  Point2f d1 = vertices.at(1) - vertices.at(0);
  Point2f d2 = vertices.at(2) - vertices.at(1);
  Point2f d3 = vertices.at(3) - vertices.at(2);
  Point2f d4 = vertices.at(0) - vertices.at(3);
  // Calculate sizes of all marker's edges.
  double dis1 = sqrt(d1.x * d1.x + d1.y * d1.y);
  double dis2 = sqrt(d2.x * d2.x + d2.y * d2.y);
  double dis3 = sqrt(d3.x * d3.x + d3.y * d3.y);
  double dis4 = sqrt(d4.x * d4.x + d4.y * d4.y);

  double smallestDistance =
      std::min(std::min(dist1, dist2), std::min(dist3, dist4));

  return distance(smallestDistance);
}
