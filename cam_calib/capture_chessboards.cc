/*
 * File name: main.cc
 * Date:      2015/06/16 08:08
 * Author:    chudoba
 */

#include <stdio.h>

#include <opencv2/opencv.hpp>

#define WINDOW_NAME "image capture"

static int mouseX = 0;
static int mouseY = 0;

void onMouse(int event, int x, int y, int flags, void* userdata);

//static float sqr(float x) { return x * x; }

int main(int argc, char** argv)
{
	int cameraId = 0;

	if (argc>1) {
		cameraId = atoi(argv[1]);
	}

	CvCapture * capture = cvCaptureFromCAM(cameraId);
	if(!capture) {
		fprintf(stderr, "Failed to open camera %d\n", cameraId);
		return -1;
	}

	if (argc>3) {
		int width = atoi(argv[2]);
		int height = atoi(argv[3]);
		fprintf(stderr, "Setting resolution to %d x %d\n", width, height);
		cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, width);
		cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, height);
	}

	cv::namedWindow(WINDOW_NAME,1);
	cv::setMouseCallback(WINDOW_NAME, onMouse, NULL);

	int saveFrameCounter = 0;
	bool enableRecognition = true;

	if (argc>4) {
		saveFrameCounter = atoi(argv[4]) - 1;
		if (saveFrameCounter < 0) saveFrameCounter = 0;
		fprintf(stderr, "Starting from frame # %d\n", saveFrameCounter+1);
	} else {
		while (true) {
			char fileName[1024];
			snprintf(fileName, sizeof(fileName), "./frame%04d.png", saveFrameCounter+1);
			FILE * f = fopen(fileName, "r");
			if (!f) {
				fprintf(stderr, "First saved file will be %s\n", fileName);
				break;
			}
			fclose(f);
			fprintf(stderr, "File %s already exists\n", fileName);
			saveFrameCounter++;
		}
	}

	bool quit = false;
	while (!quit) {
		cv::Mat frame;
		frame = cvQueryFrame(capture); // get a new frame from camera
		if (frame.empty()) break;

		//cv::Mat gray;
		//cv::cvtColor(frame, gray, CV_BGR2GRAY);
		cv::Mat displayFrame;
		frame.copyTo(displayFrame);

		if (enableRecognition) {
			cv::Size boardSize(9, 6);
			std::vector<cv::Point2f> pointBuf;
			int found = cv::findChessboardCorners( frame, boardSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			cv::drawChessboardCorners( displayFrame, boardSize, cv::Mat(pointBuf), found );
		}
	
		imshow(WINDOW_NAME, displayFrame);
		int k = cv::waitKey(30);
		switch (k) {
			case 27:
			case 'q':
			case 'Q':
				quit = true;
				break;
			case 's':
				saveFrameCounter++;
				fprintf(stderr, "save frame %d\n", saveFrameCounter);
				{
					char fileName[1024];
					snprintf(fileName, sizeof(fileName), "./frame%04d.png", saveFrameCounter);
					imwrite(fileName, frame);
				}
				break;
			case 32:
				enableRecognition = !enableRecognition;
				fprintf(stderr, "recognition %s\n", enableRecognition ? "enabled" : "disabled");
				break;
		}
	}

	cvDestroyWindow(WINDOW_NAME);
	return 0;
}

void onMouse(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == cv::EVENT_LBUTTONDOWN )
	{
		fprintf(stderr, "Left button of the mouse is clicked - position (%d, %d)\n", x, y);
	}
	else if  ( event == cv::EVENT_RBUTTONDOWN )
	{
		fprintf(stderr, "Right button of the mouse is clicked - position (%d, %d)\n", x, y);
	}
	else if  ( event == cv::EVENT_MBUTTONDOWN )
	{
		fprintf(stderr, "Middle button of the mouse is clicked - position (%d, %d)\n", x, y);
	}
	else if ( event == cv::EVENT_MOUSEMOVE )
	{
		fprintf(stderr, "Mouse move over the window - position (%d, %d)\n", x, y);
	}
	mouseX = x;
	mouseY = y;
}

/* end of main.cc */
