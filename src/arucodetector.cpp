#include <pattern_follower/arucodetector.h>

ArucoDetector::ArucoDetector(const Ptr<Dictionary> &dictionary) {
  dictionary_ = Ptr<Dictionary>(dictionary);
}

ArucoDetector::ArucoDetector(const int &count, const int &size) {
  dictionary_ = aruco::generateCustomDictionary(count, size);
}

void ArucoDetector::draw(const int &id, const int &size, Mat &output,
                         const int &border) {
  aruco::drawMarker(dictionary_, id, size, output, border);
}

void ArucoDetector::detect(Mat &input,
                           std::vector<std::vector<Point2f>> &corners,
                           std::vector<int> &ids) {
  aruco::detectMarkers(input, dictionary_, corners, ids);
}

void ArucoDetector::drawDetected(Mat &input,
                                 std::vector<std::vector<Point2f>> &corners,
                                 std::vector<int> &ids) {
  if (ids.size() > 0)
    aruco::drawDetectedMarkers(input, corners, ids);
}
