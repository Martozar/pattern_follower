#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <tuple>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include <pattern_follower/arucodetector.h>
#include <pattern_follower/cam_calibration.h>
#include <pattern_follower/camera.h>
#include <pattern_follower/camera_calibration.h>
#include <pattern_follower/contourfinder.h>
#include <pattern_follower/kalman_filter.h>
#include <pattern_follower/loader.h>
#include <pattern_follower/measurement.h>
#include <pattern_follower/parser.h>
#include <pattern_follower/robot_control.h>
#include <pattern_follower/roidetector.h>
#include <pattern_follower/templatematcher.h>
