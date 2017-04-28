#include <pattern_follower/application.h>

Application::Application(const std::string &path) {
  cv::FileStorage fs{path, FileStorage::READ};
  camera_ = std::unique_ptr<Camera>(new Camera(fs["Camera"]));
  robotControl_ = std::unique_ptr<RobotControl>(
      new RobotControl(fs["RobotControl"], (int)fs["simulation"]));
}

Application::~Application() {
  cameraThread_.join();
  robotControlThread_.join();
}

void Application::run() {
  XInitThreads();
  cameraThread_ = std::thread(&Application::cameraThreadProcess, this);
  robotControlThread_ =
      std::thread(&Application::robotControlThreadProcess, this);
}

void Application::cameraThreadProcess() {
  double dist{0.0}, angle{0.0};
  bool suceed{false};
  while (!done_) {
    suceed = camera_->proceed(angle, dist);

    mutex_.lock();
    setGlobalVariables(dist, angle, suceed);
    if (waitKey(2) >= 0)
      done_ = true;
    mutex_.unlock();
  }
}

void Application::robotControlThreadProcess() {
  double dist{0.0}, angle{0.0};
  bool suceed{false};
  while (!done_) {

    mutex_.lock();
    getGlobalVariable(dist, angle, suceed);
    if (waitKey(2) >= 0)
      done_ = true;
    mutex_.unlock();

    robotControl_->calculateRobotSpeeds(obstacles_, cv::Point2d(dist, angle),
                                        suceed);
  }
}

void Application::setLaserScanerRead(const std::vector<cv::Point2d> &data) {
  mutex_.lock();
  obstacles_ = data;
  mutex_.unlock();
}
void Application::setGlobalVariables(const double &distance,
                                     const double &angle, const bool &suceed) {
  this->dist_ = distance;
  this->angle_ = angle;
  this->suceed_ = suceed;
}

void Application::getGlobalVariable(double &distance, double &angle,
                                    bool &suceed) {
  distance = this->dist_;
  angle = this->angle_;
  suceed = this->suceed_;
}
