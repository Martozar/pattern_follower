#include "opencv2/features2d/features2d.hpp"
#include <condition_variable>
#include <iostream>
#include <pattern_follower/includes.h>
#include <queue>

#include <time.h>

int main(int argc, char **argv) {

  Parser p(argc, argv);

  if (p.has("help")) {
    p.printMessage();
    return 0;
  }

  std::string path = p.get<std::string>("config");

  Application app(path);
  app.run();
}
