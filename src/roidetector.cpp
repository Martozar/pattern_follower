#include <iostream>
#include <pattern_follower/roidetector.h>

RoiDetector::RoiDetector(const double &_threshAdapt, const int &_blockSize,
                         const int &_normSize) {
  contourFinder = std::unique_ptr<ContourFinder>(
      new ContourFinder(_threshAdapt, _blockSize));
  std::cout << "ahoj\n";
  normSize = _normSize;
  norm2dPRS.push_back(Point2f(0, 0));
  norm2dPRS.push_back(Point2f(normSize - 1, 0));
  norm2dPRS.push_back(Point2f(normSize - 1, normSize - 1));
  norm2dPRS.push_back(Point2f(0, normSize - 1));
}

void RoiDetector::normalizePattern(const Mat &src,
                                   const std::vector<Point2f> &roi, Rect &rec,
                                   Mat &dst) {
  /*Calculates a perspective transform from
                                  four pairs of the corresponding points.*/
  Mat homography(findHomography(roi, norm2dPRS, CV_RANSAC));
  Mat subImage = src(rec);
  /* Applies a perspective transformation to an inner image.*/
  warpPerspective(subImage, dst, homography, Size(dst.cols, dst.rows));
  // imshow("im", dst);
}

void RoiDetector::detectROI(const Mat &frame,
                            std::vector<std::vector<Point2f>> &refinedVertices,
                            std::vector<Mat> &regionsOfInterest) {

  Mat grayImage, binaryImage;
  std::vector<std::vector<Point>> contours;
  std::vector<Vec4i> hierarchy;
  contourFinder->binarize(frame, grayImage, binaryImage);

  contourFinder->contours(binaryImage, contours, hierarchy);

  for (int i = 0; i < contours.size(); i++) {

    int vertex = -1;
    std::vector<Point> contour = contours[i];
    std::vector<Point2f> contourApprox;

    double arcLen = arcLength(contour, true); // Calculates a contour perimeter
    approxPolyDP(contour, contourApprox, arcLen * 0.02,
                 true); // Approximates a polygonal curve

    /*We don't need too small, non-convex contours or contours at the lowest
    level of hierarchy*/
    if (std::fabs(contourArea(contour)) < 100 ||
        !isContourConvex(contourApprox) || hierarchy[i][2] == -1 ||
        contourApprox.size() != 4)
      continue;

    int averageSize = (grayImage.rows + grayImage.cols) / 2;
    double d, dMin = (4 * averageSize * averageSize);
    Point p;
    int pMinX, pMinY, pMaxX, pMaxY;
    std::vector<Point2f> roi2DPts;
    pMinX = pMaxX = contourApprox[0].x;
    pMinY = pMaxY = contourApprox[0].y;
    for (unsigned int j = 0; j < 4; j++) {
      p = contourApprox[j];
      if (p.x < pMinX)
        pMinX = p.x;
      else if (p.x > pMaxX)
        pMaxX = p.x;
      if (p.y < pMinY)
        pMinY = p.y;
      else if (p.y > pMaxY)
        pMaxY = p.y;
      d = norm(contourApprox[j]);
      if (d < dMin) {
        dMin = d;
        vertex = j;
      }
    }
    refinedVertices.push_back(contourApprox);

    Rect box(pMinX, pMinY, pMaxX - pMinX + 1, pMaxY - pMinY + 1);
    for (unsigned int j = 0; j < 4; j++) {
      roi2DPts.push_back(
          Point2f(refinedVertices.back()[(4 + vertex - j) % 4].x - pMinX,
                  refinedVertices.back()[(4 + vertex - j) % 4].y - pMinY));
    }
    Mat normROI = Mat(normSize, normSize, CV_8UC1);
    normalizePattern(grayImage, roi2DPts, box, normROI);
    regionsOfInterest.push_back(normROI);
  }
}
