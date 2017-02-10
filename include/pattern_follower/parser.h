#ifndef PARSER_H
#define PARSER_H

#include <opencv2/opencv.hpp>

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

class Parser : public cv::CommandLineParser {
public:
  Parser(int argc, char **argv) : cv::CommandLineParser(argc, argv, keys){};
  virtual ~Parser(){};

protected:
private:
};

#endif // PARSER_H
