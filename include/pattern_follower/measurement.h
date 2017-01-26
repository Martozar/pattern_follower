#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace cv;

class Measurement {
public:
  Measurement(const int &frame_size, const double &pattern_width_cm,
              const int &distance, const int &pattern_width_pix,
              const double &fovx) {
    center = frame_size / 2;
    pattern_width = pattern_width_cm;
    focal_length = ((double)pattern_width_pix * (double)distance) /
                   ((double)pattern_width);
    anglePerPixel = fovx / frame_size;
  }

  Measurement(const int &frame_size, const double &pattern_width_cm,
              const int &distance, const int &pattern_width_pix) {
    center = frame_size / 2;
    pattern_width = pattern_width_cm;
    focal_length = ((double)pattern_width_pix * (double)distance) /
                   ((double)pattern_width);

    double fovx = 2 * atan((2 * focal_length) / frame_size);
    anglePerPixel = fovx / frame_size;
  }

  double distance(const int &pattern_width_pix) const;

  double distance(const std::vector<Point2f> &vertices) const;

  double angle(const std::vector<Point2f> &vertices) const;

protected:
private:
  int center;
  double pattern_width;
  double focal_length;
  double anglePerPixel;
};

#endif // MEASUREMENT_H
