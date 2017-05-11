#ifndef PARSER_H
#define PARSER_H

#include <opencv2/opencv.hpp>

const cv::String keys = "{help h usage ?||print this message}"
                        "{config c|../config.yaml|path to config file}";

class Parser : public cv::CommandLineParser {
public:
  Parser(int argc, char **argv) : cv::CommandLineParser(argc, argv, keys){};
  virtual ~Parser(){};

protected:
private:
};

#endif // PARSER_H
