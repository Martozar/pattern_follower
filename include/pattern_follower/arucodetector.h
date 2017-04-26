#ifndef ARUCODETECTOR_H
#define ARUCODETECTOR_H

#include <chrono>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <pattern_follower/detector.h>

using namespace cv;
using namespace aruco;

class ArucoDetector : public Detector {
public:
  ArucoDetector(
      const Dictionary &dictionary = getPredefinedDictionary(DICT_4X4_100)) {
    dictionary_ = dictionary;
  };

  ArucoDetector(const int &count, const int &size) {
    dictionary_ = aruco::generateCustomDictionary(count, size);
  }

  void draw(const int &id, const int &size, Mat &output, const int &border) {
    aruco::drawMarker(dictionary_, id, size, output, border);
  }

  virtual void detect(Mat &input, std::vector<std::vector<Point2f>> &corners,
                      std::vector<int> &ids) override {
    aruco::detectMarkers(input, dictionary_, corners, ids);
  }

  void drawDetected(Mat &input, std::vector<std::vector<Point2f>> &corners,
                    std::vector<int> &ids) override {
    if (ids.size() > 0)
      aruco::drawDetectedMarkers(input, corners, ids);
  }

protected:
private:
  Dictionary dictionary_;
};

#endif // ARUCODETECTOR_H
