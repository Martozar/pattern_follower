#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace cv;

class Measurement {
public:
  Measurement();

  Measurement(const int &frameSize, const double &patternWidthCm,
              const int &distance, const int &pattern_widthPix);

  Measurement(const int &frameSize, const double &patternWidthCm,
              const double &distance, const double &patternWidthPix,
              const double &fovx);

  double distance(const int &pattern_width_pix) const;

  double distance(const std::vector<Point2f> &vertices) const;

  double angle(const std::vector<Point2f> &vertices) const;

protected:
private:
  int center_;
  double patternWidth_;
  double focalLength_;
  double anglePerPixel_;
};

#endif // MEASUREMENT_H
