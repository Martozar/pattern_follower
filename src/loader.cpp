#include <pattern_follower/loader.h>

bool loadImages(const cv::String &path, const int &size,
                std::vector<cv::Mat> &data) {
  std::vector<cv::String> images;
  glob(path, images);
  for (unsigned int i = 0; i < images.size(); i++) {
    if (!loadPattern(images[i], size, data)) {
      data.clear();
      return false;
    }
  }
  return true;
}

bool loadPattern(const cv::String &filename, const int &size,
                 std::vector<cv::Mat> &library) {
  cv::Mat img = imread(filename, 0);
  if (img.cols != img.rows) {
    return false;
  }

  cv::Mat src(size, size, CV_8UC1);
  cv::resize(img, src, cv::Size(size, size));

  cv::Mat subImg =
      src(cv::Range(size / 5, 4 * size / 5), cv::Range(size / 5, 4 * size / 5));
  library.push_back(subImg);

  return true;
}
