#include <pattern_follower/templatematcher.h>

TemplateMatcher::TemplateMatcher() {
  roiDetector_ = std::unique_ptr<RoiDetector>(new RoiDetector);
  loadImages("/home/michail/pattern_follower/patterns/*.png", NORM_PATTERN_SIZE,
             library_);
  confThreshold_ = CONF_TRESH;
  normSize_ = NORM_PATTERN_SIZE;
}

bool TemplateMatcher::identifyPattern(const Mat &src, patInfo &out) {
  double correlation;

  Mat copy;
  src.copyTo(copy);
  Mat inter = copy(Range(normSize_ / 4, 3 * normSize_ / 4),
                   Range(normSize_ / 4, 3 * normSize_ / 4));
  imshow("src", inter);
  imshow("im", library_[0]);
  out.maxCor = -1.0;

  for (unsigned int i = 0; i < library_.size(); i++) {

    for (unsigned int j = 0; j < 4; j++) {
      correlation = this->correlation(inter, library_[i]);
      std::cout << "cor " << correlation << "\n";
      if (correlation > out.maxCor) {

        out.maxCor = correlation;
        out.index = i + 1;
        if (out.maxCor > confThreshold_) {
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
  std::vector<std::vector<Point2f>> refinedVertices;
  std::vector<Mat> ROI;

  roiDetector_->detectROI(frame, refinedVertices, ROI);
  for (unsigned int i = 0; i < ROI.size(); i++) {
    patInfo out;
    if (identifyPattern(ROI[i], out)) {
      corners.push_back(refinedVertices[i]);
      if (out.ori != 0) {
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

void TemplateMatcher::drawDetected(
    cv::Mat &frame, std::vector<std::vector<cv::Point2f>> &corners,
    std::vector<int> &ids) {
  for (unsigned int i = 0; i < ids.size(); i++) {
    Mat currentMarker = Mat(corners[i]);
    for (unsigned int j = 0; j < 4; j++) {
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
