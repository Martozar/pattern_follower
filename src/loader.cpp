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
      src(cv::Range(size / 4, 3 * size / 4), cv::Range(size / 4, 3 * size / 4));

  for (int i = 0; i < 4; i++) {
    library.push_back(subImg);
    cv::transpose(subImg, subImg);
    cv::flip(subImg, subImg, 1);
  }

  return true;
}
