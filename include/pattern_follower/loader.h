#ifndef LOADER_H
#define LOADER_H

#include <opencv2/opencv.hpp>

class Loader {
public:
  Loader(){};
  virtual ~Loader(){};
  bool loadImages(const cv::String &path, const int &size,
                  std::vector<cv::Mat> &data);

protected:
private:
  bool loadPattern(const cv::String &filename, const int &size,
                   std::vector<cv::Mat> &library);
};

#endif // LOADER_H
