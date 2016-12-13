#ifndef ROIDETECTOR_H
#define ROIDETECTOR_H


#include "pattern.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace cv;

class RoiDetector
{
public:
    RoiDetector(const double _threshAdapt, const int _blockSize, const int _normSizes);
    virtual ~RoiDetector(){};
    void detectROI(const std::vector<Point>  & contours, const std::vector<Vec4i> & hierarchy,  const int index,const Mat & grayImage, std::vector<Point2f> & refinedVertices, int & vertex);
    Mat & getNormROI() {return normROI;};
protected:

private:
    int  normSize, blockSize;
    double threshAdapt;
    Mat normROI;
    Point2f norm2dPRS[4];

    void binarize(const Mat & src, Mat & image_gray, Mat & dst);
    void normalizePattern(const Mat & src, const Point2f roi[], Rect & rec, Mat & dst);
};

#endif // PATTERDETECTOR_H
