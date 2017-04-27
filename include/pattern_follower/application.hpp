#ifndef APPLICATION_H
#define APPLICATION_H
#include <chrono>
#include <pattern_follower/camera.hpp>
#include <pattern_follower/robot_control.hpp>
#include <thread>

class Application {
public:
  Application(const cv::FileStorage &fs);
  ~Application();
  void run();

private:
  void cameraThreadProcess();
  void robotControlThreadProcess();

  void setGlobalVariables(const double &distance, const double &angle,
                          const bool &suceed);
  void getGlobalVariable(double &distance, double &angle, bool &suceed);

  std::mutex mutex_;

  std::unique_ptr<Camera> camera_;
  std::unique_ptr<RobotControl> robotControl_;

  std::thread cameraThread_;
  std::thread robotControlThread_;

  double dist_{80.0}, angle_{0.0};
  bool suceed_{false};
};
#endif
