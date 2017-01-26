#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <tuple>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <pattern_follower/arucodetector.h>
#include <pattern_follower/cam_calibration.h>
#include <pattern_follower/camera.h>
#include <pattern_follower/camera_calibration.h>
#include <pattern_follower/contourfinder.h>
#include <pattern_follower/measurement.h>
#include <pattern_follower/parser.h>
#include <pattern_follower/pattern.h>
#include <pattern_follower/pid.h>
#include <pattern_follower/robot.h>
#include <pattern_follower/roidetector.h>
#include <pattern_follower/templatematcher.h>

#define MAX_VEL_REAL 200
#define MAX_ANG_VEL_REAL 100
#define ZRYCHLENI_REAL 10

#define MAX_VEL_SIM 20
#define MAX_ANG_VEL_SIM 5
#define ZRYCHLENI_SIM 1

#define DIST_KP 5.0
#define DIST_KI 0.0
#define DIST_KD 1.0

#define ANGLE_KP 3.0
#define ANGLE_KI 0.0
#define ANGLE_KD 1.0

char *IP = "172.43.50.193";
int port = 50004;

const cv::String keys =
    "{help h usage ?||print this message}"
    "{calibration calibrate calib c||run calibration}"
    "{calibration_params cp|../calibration/default.xml|path to calibration "
    "parameters file}"
    "{param_path params        |../calibration/out_camera_data.xml| path to "
    "calibration camera parameters   }"
    "{patterns p       |../patterns/*.png     | path to patterns               "
    "}"
    "{simulation s||enable simulation}";

using namespace cv;

bool loadPattern(const String &filename, std::vector<Mat> &library) {
  Mat img = imread(filename, 0);
  if (img.cols != img.rows) {
    return false;
  }

  int msize = 80;

  Mat src(msize, msize, CV_8UC1);
  Point2f center((msize - 1) / 2.0f, (msize - 1) / 2.0f);
  Mat rot_mat(2, 3, CV_32F);
  resize(img, src, Size(msize, msize));

  Mat subImg =
      src(Range(msize / 4, 3 * msize / 4), Range(msize / 4, 3 * msize / 4));
  library.push_back(subImg);

  rot_mat = getRotationMatrix2D(center, msize, 1.0);

  for (int i = 1; i < 2; i++) {
    Mat dst = Mat(msize, msize, CV_8UC1);
    rot_mat = getRotationMatrix2D(center, -i * 90, 1.0);
    warpAffine(src, dst, rot_mat, Size(msize, msize));
    Mat subImg =
        dst(Range(msize / 4, 3 * msize / 4), Range(msize / 4, 3 * msize / 4));
    library.push_back(subImg);
  }
  return true;
}

bool loadImages(const String &path, std::vector<Mat> &data) {
  std::vector<String> images;
  glob(path, images);
  for (int i = 0; i < images.size(); i++) {
    if (!loadPattern(images.at(i), data)) {
      return false;
    }
  }
  return true;
}
