#include <iostream>
#include <pattern_follower/roidetector.h>

RoiDetector::RoiDetector(const double &_threshAdapt, const int &_blockSize,
                         const int &_normSize) {
  threshAdapt = _threshAdapt;
  blockSize = _blockSize;
  normSize = _normSize;
  normROI = Mat(normSize, normSize, CV_8UC1);

  norm2dPRS[0] = Point2f(0, 0);
  norm2dPRS[1] = Point2f(normSize - 1, 0);
  norm2dPRS[2] = Point2f(normSize - 1, normSize - 1);
  norm2dPRS[3] = Point2f(0, normSize - 1);
}

void RoiDetector::binarize(const Mat &src, Mat &image_gray, Mat &dst) {
  if (src.channels() == 3)
    cvtColor(src, image_gray, CV_BGR2GRAY);
  else
    src.copyTo(image_gray);

  adaptiveThreshold(image_gray, dst, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
                    CV_THRESH_BINARY_INV, blockSize, threshAdapt);

  dilate(dst, dst, Mat());
}

void RoiDetector::normalizePattern(const Mat &src, const Point2f roi[],
                                   Rect &rec, Mat &dst) {

  Mat homography(getPerspectiveTransform(roi, norm2dPRS)); // Calculates a
                                                           // perspective
                                                           // transform from
                                                           // four pairs of the
                                                           // corresponding
                                                           // points.

  Mat subImage =
      src(Range(rec.y, rec.y + rec.height), Range(rec.x, rec.x + rec.width));
  warpPerspective(
      subImage, dst, homography,
      Size(
          dst.cols,
          dst.rows)); // Applies a perspective transformation to an inner image.
}

void RoiDetector::detectROI(const std::vector<Point> &contour,
                            const std::vector<Vec4i> &hierarchy,
                            const int index, const Mat &grayImage,
                            std::vector<Point2f> &refinedVertices,
                            int &vertex) {

  Point2f roi2DPts[4];
  int averageSize = (grayImage.rows + grayImage.cols) / 2;

  Point p;
  int pMinX, pMinY, pMaxX, pMaxY;
  std::vector<Point> contourApprox;

  double arcLen = arcLength(contour, true); // Calculates a contour perimeter
  approxPolyDP(contour, contourApprox, arcLen * 0.02,
               true); // Approximates a polygonal curve
  if (fabs(contourArea(contour)) < 100 || !isContourConvex(contourApprox) ||
      hierarchy[index][2] == -1) // We don't need too small, non-convex contours
                                 // or contours at the lowest level of hierarchy
    return;
  if (contourApprox.size() == 4) {
    pMinX = pMaxX = contourApprox.at(0).x;
    pMinY = pMaxY = contourApprox.at(0).y;
    double d;
    double dMin = (4 * averageSize * averageSize);
    for (int j = 0; j < 4; j++) {
      p = contourApprox.at(j);
      if (p.x < pMinX)
        pMinX = p.x;
      else if (p.x > pMaxX)
        pMaxX = p.x;
      if (p.y < pMinY)
        pMinY = p.y;
      else if (p.y > pMaxY)
        pMaxY = p.y;

      d = norm(contourApprox.at(j));
      if (d < dMin) {
        dMin = d;
        vertex = j;
      }
      refinedVertices.push_back(contourApprox.at(j));
    }

    Rect box(pMinX, pMinY, pMaxX - pMinX + 1, pMaxY - pMinY + 1);

    cornerSubPix(grayImage, refinedVertices, Size(3, 3), Size(-1, -1),
                 TermCriteria(1, 3, 1));

    for (int j = 0; j < 4; j++) {
      roi2DPts[j] = Point2f(refinedVertices.at((4 + vertex - j) % 4).x - pMinX,
                            refinedVertices.at((4 + vertex - j) % 4).y - pMinY);
    }

    normalizePattern(grayImage, roi2DPts, box, normROI);
  }
}
