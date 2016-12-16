#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <tuple>

#include <opencv2/opencv.hpp>

#include <pattern_follower/contourfinder.h>
#include <pattern_follower/roidetector.h>
#include <pattern_follower/pattern.h>
#include <pattern_follower/tamplatematcher.h>
#include <pattern_follower/arucodetector.h>
#include <pattern_follower/measurement.h>
#include <pattern_follower/robot.h>
#include <pattern_follower/pid.h>
#include <pattern_follower/cam_calibration.h>
#include <pattern_follower/parser.h>
#include <pattern_follower/camera_calibration.h>


#define SIMULATION 1

#define MAX_VEL 200
#define MAX_ANG_VEL 10
#define ZRYCHLENI 1
#define DIST_KP 5.0
#define DIST_KI 0.0
#define DIST_KD 1.0

#define ANGLE_KP 3.0
#define ANGLE_KI 0.0
#define ANGLE_KD 1.0

char * IP = "10.10.40.213";
int port = 5566;

const cv::String keys =
    "{help h usage ? |      | print this message   }"
    "{calibration calibrate calib c       |      | run calibration   }"
    "{calibration_params cp|../calibration/default.xml|path to calibration parameters file}"
    "{param_path params        |../calibration/out_camera_data.xml| path to calibration camera parameters   }"
    "{patterns p       |../patterns/*.png     | path to patterns               }"
    "{simulation s||enable simulation}"
    ;

using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::ml;
using namespace std;




void detectAruco(Mat & frame, ArucoDetector & arucoDetector, Measurement & measurement, double & angle, double & distance)
{
  std::vector<std::vector<Point2f>> cor;
  std::vector<int> ids;
  arucoDetector.detect(frame, cor, ids);

  if(ids.size() > 0)
  {
    arucoDetector.drawDetected(frame, cor, ids);
    angle = measurement.angle(cor.at(0));
    distance = measurement.distance(cor.at(0));
  }
}

tuple<double, double> runAruco(Mat & frame, const Mat & cameraMatrix, const Mat & distCoeffs, ArucoDetector & arucoDetector, Measurement & measurement, PID & angleController, PID & distanceController, double & angle, double & distance, const double & dt = 1.0)
{
  std::vector<std::vector<Point2f>> cor;
  std::vector<int> ids;
  detectAruco(frame, arucoDetector, measurement, angle, distance);
  return make_tuple(angleController.calculate(0, angle, dt), distanceController.calculate(40, distance, dt));
}

int loadPattern(const String & filename, std::vector<cv::Mat> & library, int & patternCount)
{
    Mat img = imread(filename,0);
    if(img.cols!=img.rows)
    {
        return -1;
    }

    int msize = 80;

    Mat src(msize, msize, CV_8UC1);
    Point2f center((msize-1)/2.0f,(msize-1)/2.0f);
    Mat rot_mat(2,3,CV_32F);
    resize(img, src, Size(msize,msize));

    Mat subImg = src(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));
    library.push_back(subImg);

    rot_mat = getRotationMatrix2D( center, msize, 1.0);


    for (int i=1; i<4; i++)
    {
        Mat dst= Mat(msize, msize, CV_8UC1);
        rot_mat = getRotationMatrix2D( center, -i*90, 1.0);
        warpAffine( src, dst, rot_mat, Size(msize,msize));
        Mat subImg = dst(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));
        library.push_back(subImg);
    }

    patternCount++;
    return 1;
}

void loadImages(const String & path, std::vector<Mat> & data)
{
    std::vector<String> images;
    glob(path, images);
    for(int i = 0; i < images.size(); i++)
    {
        loadPattern(images.at(i), data, i);
    }
}
