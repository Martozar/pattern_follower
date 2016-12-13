#ifndef TAMPLATEMATCHER_H
#define TAMPLATEMATCHER_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "pattern.h"

using namespace cv;

class TamplateMatcher
{
public:
    TamplateMatcher(const std::vector<Mat> & _library, const double _confThreshold, const int _normSize)
    {
        library = _library;
        confThreshold = _confThreshold;
        normSize = _normSize;
    };
    virtual void detect(const Mat & src, const Mat & cameraMatrix, const Mat & distortions, std::vector<Point2f> & refinedVertices, const int & vertex, std::vector<Pattern> & foundPatterns);

    virtual ~TamplateMatcher() {};
protected:

private:
    struct patInfo
    {
        int index;
        int ori;
        double maxCor;
    };
    int normSize;
    double confThreshold;
    std::vector<Mat> library;
    Mat distortions, cameraMatrix;
    bool identifyPattern(const Mat & src, patInfo & out);


};

#endif // TAMPLATEMATCHER_H
