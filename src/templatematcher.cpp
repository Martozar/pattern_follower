#include <pattern_follower/templatematcher.h>

bool TemplateMatcher::identifyPattern(const Mat &src, patInfo &out) {
  double correlation;

  Mat copy;
  src.copyTo(copy);
  Mat inter = copy(Range(normSize / 4, 3 * normSize / 4),
                   Range(normSize / 4, 3 * normSize / 4));

  out.maxCor = -1.0;

  for (unsigned int i = 0; i < library.size(); i++) {

    for (unsigned int j = 0; j < 4; j++) {
      correlation = this->correlation(inter, library.at(i));
      // std::cout << correlation << "\n";
      if (correlation > out.maxCor) {
        out.maxCor = correlation;
        out.index = i + 1;
        if (out.maxCor > confThreshold) {
          out.ori = j;
          return true;
        }
      }
      transpose(inter, inter);
      flip(inter, inter, 1);
    }
  }
  return false;
}

void TemplateMatcher::detect(Mat &frame,
                             std::vector<std::vector<Point2f>> &corners,
                             std::vector<int> &ids) {
  Mat grayImage, binaryImage;
  std::vector<std::vector<Point>> contours;
  std::vector<Point2f> refinedVertices;
  std::vector<Vec4i> hierarchy;
  int vertex = -1;
  contourFinder.binarize(frame, grayImage, binaryImage);
  contourFinder.contours(binaryImage, contours, hierarchy);
  for (unsigned int i = 0; i < contours.size(); i++) {

    roiDetector.detectROI(contours[i], hierarchy, i, grayImage, refinedVertices,
                          vertex);
    if (refinedVertices.size() == 4) {
      patInfo out;
      if (identifyPattern(roiDetector.getNormROI(), out)) {

        corners.push_back(refinedVertices);
        if (out.ori != 0) {
          std::cout << out.ori << std::endl;
          int last = corners.size() - 1;
          std::vector<Point2f> cornersCopy = corners[last];
          for (int i = 0; i < 4; i++) {
            corners[last][i] = cornersCopy[(i + 4 - out.ori) % 4];
          }
        }

        ids.push_back(out.index);
      }
    }
  }
}

void TemplateMatcher::drawDetected(
    cv::Mat &frame, std::vector<std::vector<cv::Point2f>> &corners,
    std::vector<int> &ids) {
  for (unsigned int i = 0; i < ids.size(); i++) {
    Mat currentMarker = Mat(corners[i]);
    for (int j = 0; j < 4; j++) {
      Point2f p0, p1;
      p0 = currentMarker.ptr<Point2f>(0)[j];
      p1 = currentMarker.ptr<Point2f>(0)[(j + 1) % 4];
      line(frame, p0, p1, Scalar(0, 255, 0), 3);
    }
    rectangle(frame, currentMarker.ptr<Point2f>(0)[0] - Point2f(3, 3),
              currentMarker.ptr<Point2f>(0)[0] + Point2f(3, 3),
              Scalar(255, 0, 0), 1, LINE_AA);
  }
}

double TemplateMatcher::correlation(cv::Mat &image_1, cv::Mat &image_2) {

  // convert data-type to "float"
  cv::Mat im_float_1;
  image_1.convertTo(im_float_1, CV_32F);
  cv::Mat im_float_2;
  image_2.convertTo(im_float_2, CV_32F);

  int n_pixels = im_float_1.rows * im_float_1.cols;

  // Compute mean and standard deviation of both images
  cv::Scalar im1_Mean, im1_Std, im2_Mean, im2_Std;
  meanStdDev(im_float_1, im1_Mean, im1_Std);
  meanStdDev(im_float_2, im2_Mean, im2_Std);

  // Compute covariance and correlation coefficient
  double covar = (im_float_1 - im1_Mean).dot(im_float_2 - im2_Mean) / n_pixels;
  double correl = covar / (im1_Std[0] * im2_Std[0]);

  return correl;
}
