/*
 * File name: cam_calibration.cc
 * Date:      2013/11/20 16:27
 * Author:    Jan Chudoba
 */

#include <pattern_follower/cam_calibration.h>

CCameraCalibration * CCameraCalibration::instance = NULL;
int CCameraCalibration::instanceCount = 0;

CCameraCalibration::CCameraCalibration()
{
	valid = false;
}

CCameraCalibration::~CCameraCalibration()
{
	if (this == instance) {
		instance = NULL;
	}
}

bool CCameraCalibration::readFromFile(const cv::String fileName)
{
#ifndef NO_OPENCV
	cv::FileStorage fs(std::string(fileName), cv::FileStorage::READ);
	read(fs);
	fs.release();
	return true;
#else
	ERROR("CCameraCalibration::readFromFile() not available, because opencv is disabled");
	return false;
#endif
}

#ifndef NO_OPENCV
void CCameraCalibration::read(const cv::FileStorage& node)
{
	nrOfFrames = (int)node["nrOfFrames"];
	imageWidth = (int)node["image_Width"];
	imageHeight = (int)node["image_Height"];
	node["Camera_Matrix"] >> cameraMatrix;
	node["Distortion_Coefficients"] >> distortionCoefficients;
	avgReprojectionError = (double)node["Avg_Reprojection_Error"];
	//valid = (cameraMatrix.rows == 3 && cameraMatrix.cols == 3);
}
#endif

CCameraCalibration * CCameraCalibration::getInstance(const char * cfgFile)
{
	if (!instance) {
		instance = new CCameraCalibration();
		if (cfgFile) {
			instance->readFromFile(cfgFile);
		}
	}
	instanceCount++;
	return instance;
}

void CCameraCalibration::releaseInstance(CCameraCalibration * i)
{
	if (i == instance) {
		instanceCount --;
		if (instanceCount == 0) {
			delete instance;
			instance = NULL;
		}
	}
}

/* end of cam_calibration.cc */
