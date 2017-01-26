/*
 * File name: cam_calibration.h
 * Date:      2013/11/20 16:18
 * Author:    Jan Chudoba
 */

#ifndef __CAM_CALIBRATION_H__
#define __CAM_CALIBRATION_H__

#include <stdlib.h>
#ifndef NO_OPENCV
#include <opencv2/core/core.hpp>
#else
#include "fake_opencv.h"
#endif

class CCameraCalibration {
public:
  bool valid;
  int nrOfFrames;
  int imageWidth;
  int imageHeight;
  cv::Mat cameraMatrix;
  cv::Mat distortionCoefficients;
  double avgReprojectionError;

public:
  CCameraCalibration();
  ~CCameraCalibration();

  bool readFromFile(const cv::String fileName);
#ifndef NO_OPENCV
  void read(const cv::FileStorage &node);

  double getFocalLengthX() { return cameraMatrix.at<double>(0, 0); }
  double getFocalLengthY() { return cameraMatrix.at<double>(1, 1); }
  double getPrincipalPointX() { return cameraMatrix.at<double>(0, 2); }
  double getPrincipalPointY() { return cameraMatrix.at<double>(1, 2); }
#else
  double getFocalLengthX() { return 0; }
  double getFocalLengthY() { return 0; }
  double getPrincipalPointX() { return 0; }
  double getPrincipalPointY() { return 0; }
#endif

  static CCameraCalibration *getInstance(const char *cfgFile = NULL);
  static void releaseInstance(CCameraCalibration *i);

private:
  static CCameraCalibration *instance;
  static int instanceCount;
};

#endif

/* end of cam_calibration.h */
