#ifndef APPLICATION_H
#define APPLICATION_H
#include <chrono>
#include <flexiport/flexiport.h>
#include <hokuyoaist/hokuyo_errors.h>
#include <hokuyoaist/hokuyoaist.h>
#include <mutex>
#include <pattern_follower/camera.h>
#include <pattern_follower/robot_control.h>
#include <thread>

#include <X11/Xlib.h>

const int SIZE = 625;
const double STEP = 0.006135920;
const double FIRST = -1.91986;
const double LAST = -FIRST;
class Application {
public:
  Application(const std::string &path);
  ~Application();
  void run();

private:
  void cameraThreadProcess();
  void robotControlThreadProcess();
  void dataReadThreadProcess();

  void setGlobalVariables(const double &distance, const double &angle,
                          const bool &suceed);
  void getGlobalVariable(double &distance, double &angle, bool &suceed);

  void setLaserScanerRead(const std::vector<cv::Point2d> &data);

  std::mutex mutex_;

  std::unique_ptr<Camera> camera_;
  std::unique_ptr<RobotControl> robotControl_;

  std::thread cameraThread_;
  std::thread robotControlThread_;
  std::thread dataThread_;

  hokuyoaist::Sensor laser;

  double dist_{80.0}, angle_{0.0};
  bool suceed_{false}, done_{false};
  int isLaser;
  std::vector<cv::Point2d> obstacles_;
};
#endif
