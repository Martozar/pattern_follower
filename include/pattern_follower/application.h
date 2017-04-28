#ifndef APPLICATION_H
#define APPLICATION_H
#include <chrono>
#include <mutex>
#include <pattern_follower/camera.h>
#include <pattern_follower/robot_control.h>

#include <X11/Xlib.h>
#include <thread>

class Application {
public:
  Application(const std::string &path);
  ~Application();
  void run();

private:
  void cameraThreadProcess();
  void robotControlThreadProcess();

  void setGlobalVariables(const double &distance, const double &angle,
                          const bool &suceed);
  void getGlobalVariable(double &distance, double &angle, bool &suceed);

  void setLaserScanerRead(const std::vector<cv::Point2d> &data);

  std::mutex mutex_;

  std::unique_ptr<Camera> camera_;
  std::unique_ptr<RobotControl> robotControl_;

  std::thread cameraThread_;
  std::thread robotControlThread_;

  double dist_{80.0}, angle_{0.0};
  bool suceed_{false}, done_{false};
  std::vector<cv::Point2d> obstacles_;
};
#endif
