#ifndef PATTERN_H
#define PATTERN_H

//#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace cv;

class Pattern
{
    public:
        Pattern(double _size = 90);
        virtual ~Pattern(){};

        void rotationMatrix(const Mat& rot_vec, Mat& rot_mat);
        void getExtrinsics(int patternSize, const Mat& cameraMatrix, const Mat& distortions);
        void setID(int _id){id = _id;};
        void setOrientation(int _orientation){orientation = _orientation;};
        void setConfidence(double _confidence){confidence = _confidence;};
        float getSize() {return size;};
        std::vector<Point2f> & getVetices(){return vertices;};
        void draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix);

    protected:

    private:
        std::vector<Point2f> vertices;
        float size; //size of pattern in millimeters
        double confidence;
        int id;
        int orientation;
        Mat rotationMat, rotationVec, transitionVec;
};

#endif // PATTERN_H
