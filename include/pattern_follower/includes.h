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
#include <pattern_follower/kalman_filter.h>
#include <pattern_follower/loader.h>
#include <pattern_follower/map.h>
#include <pattern_follower/measurement.h>
#include <pattern_follower/parser.h>
#include <pattern_follower/pid.h>
#include <pattern_follower/robot.h>
#include <pattern_follower/roidetector.h>
#include <pattern_follower/templatematcher.h>

#define NORM_PATTERN_SIZE 80
#define FRAME_SIZE 640
#define ADAPT_BLOCK_SIZE 45
#define ADAPT_TRESH 5.0
#define CONF_TRESH 0.75

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

// using namespace cv;
