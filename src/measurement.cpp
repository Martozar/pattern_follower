#include <pattern_follower/measurement.h>

Measurement::Measurement(const FileNode &fn) {
  int frameSize = (int)fn["frame_size"];
  center_ = frameSize / 2;
  patternWidth_ = (double)fn["pattern_width_cm"];
  double distance = (double)fn["distance"];
  double patternWidthPix = (double)fn["patter_width_pix"];
  focalLength_ = patternWidthPix * distance / patternWidth_;
  double fovx = 2 * atan(frameSize / (2 * focalLength_));
  anglePerPixel_ = fovx / frameSize;
}

Measurement::Measurement(const int &frameSize, const double &patternWidthCm,
                         const double &distance, const double &patternWidthPix,
                         const double &fovx) {
  center_ = frameSize / 2;
  patternWidth_ = patternWidthCm;
  focalLength_ =
      ((double)patternWidthPix * (double)distance) / ((double)patternWidth_);
  anglePerPixel_ = fovx / frameSize;
}

double Measurement::distance(const int &patternWidthPix) const {
  return (patternWidth_ * focalLength_) / ((double)patternWidthPix);
}

double Measurement::angle(const std::vector<Point2f> &vertices) const {
  Moments mu = moments(vertices, false);
  Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

  return (center_ - mc.x) * anglePerPixel_;
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
