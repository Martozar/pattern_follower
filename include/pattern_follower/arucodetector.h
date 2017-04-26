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
  ArucoDetector(const Ptr<Dictionary> &dictionary =
                    getPredefinedDictionary(DICT_4X4_100));

  ArucoDetector(const int &count, const int &size);

  void draw(const int &id, const int &size, Mat &output, const int &border);

  virtual void detect(Mat &input, std::vector<std::vector<Point2f>> &corners,
                      std::vector<int> &ids) override;

  virtual void drawDetected(Mat &input,
                            std::vector<std::vector<Point2f>> &corners,
                            std::vector<int> &ids) override;

private:
  Ptr<Dictionary> dictionary_;
};

#endif // ARUCODETECTOR_H
