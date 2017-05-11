/**
 * @file measurement.h
 * @breaf Calculate position of marker against camera.
 *
 * @author Mykhaylo Zelenskyy
 * @version 1.0
 */

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace cv;

class Measurement {
public:
  Measurement(const FileNode &fn);

  Measurement(const int &frameWidth, const double &patternWidth,
              const double &distance, const double &patternWidthPix,
              const double &fovx);

  /**
   * Calculates distance from marker to camera.
   * @param [in] pattern_width_pix size of detected marker in pixels.
   * @return distance from marker to camera.
   * @see #distance
   */
  double distance(const double &pattern_width_pix) const;

  /**
   * Finds the smallest size of marker and calculate distance from marker to
   * camera.
   * @param [in] vertices marker's corners' position..
   * @return distance from marker to camera.
   * @see #distance
   */

  double distance(const std::vector<Point2f> &vertices) const;

  /**
   * Calculates angle between camera and marker.
   * @param [in] vertices marker's corners' position..
   * @return angle between camera and marker.
   */
  double angle(const std::vector<Point2f> &vertices) const;

protected:
private:
  int center_;
  double patternWidth_;
  double focalLength_;
  double anglePerPixel_;
};

#endif // MEASUREMENT_H
