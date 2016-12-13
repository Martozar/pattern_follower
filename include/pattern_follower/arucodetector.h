#ifndef ARUCODETECTOR_H
#define ARUCODETECTOR_H

#include <opencv2/aruco.hpp>
#include <iostream>

using namespace cv;
using namespace aruco;


class ArucoDetector
{
public:
    ArucoDetector(const Ptr<Dictionary> & _dictionary = Ptr<Dictionary>(getPredefinedDictionary(DICT_4X4_100)))
    {
        dictionary = _dictionary;
    };

    virtual ~ArucoDetector() {};


    ArucoDetector(const int & count, const int & size)
    {
        dictionary = aruco::generateCustomDictionary(count, size);
    }

    void draw(const int & id, const int & size, Mat & output, const int & border)
    {
        aruco::drawMarker(dictionary, id, size, output, border);

    }

    virtual void detect(Mat & input, std::vector<std::vector<Point2f>> & corners, std::vector<int> & ids)
    {
        aruco::detectMarkers(input, dictionary, corners, ids);

    }

    void drawDetected(Mat & input, std::vector<std::vector<Point2f>> & corners, std::vector<int> & ids)
    {
        if(ids.size() > 0)
            aruco::drawDetectedMarkers(input, corners, ids);
    }


protected:

private:

    Ptr<Dictionary> dictionary;
};

#endif // ARUCODETECTOR_H
