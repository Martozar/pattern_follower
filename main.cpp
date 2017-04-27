#include "opencv2/features2d/features2d.hpp"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <pattern_follower/includes.h>
#include <queue>

#include <time.h>

int main(int argc, char **argv) {

  std::string path = "/home/michail/pattern_follower/config.yaml";
  if (argc > 1)
    path = argv[1];
  FileStorage fs(path, FileStorage::READ);
  Mat grayImage, binaryImage, frame;

  Application app(fs);
  app.run();
}
