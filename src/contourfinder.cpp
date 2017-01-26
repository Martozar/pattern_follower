#include <pattern_follower/contourfinder.h>

void ContourFinder::contours(const Mat &image,
                             std::vector<std::vector<Point>> &contours,
                             std::vector<Vec4i> &hierarchy) const {
  findContours(image, contours, hierarchy, CV_RETR_TREE,
               CV_CHAIN_APPROX_SIMPLE);
}

void ContourFinder::binarize(const Mat &image, Mat &grayImage,
                             Mat &binaryImage) {
  if (image.channels() == 3)
    cvtColor(image, grayImage, CV_BGR2GRAY);
  else
    image.copyTo(grayImage);

  adaptiveThreshold(grayImage, binaryImage, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
                    CV_THRESH_BINARY_INV, blockSize, threshold);

  dilate(binaryImage, binaryImage, Mat());
}
