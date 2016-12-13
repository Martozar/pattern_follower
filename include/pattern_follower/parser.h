#ifndef PARSER_H
#define PARSER_H

#include <opencv2/opencv.hpp>



class Parser : public cv::CommandLineParser
{
public:
    Parser(int argc, char** argv, const cv::String keys) : cv::CommandLineParser(argc, argv, keys) {};
    virtual ~Parser() {};

protected:

private:

};

#endif // PARSER_H
