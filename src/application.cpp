#include <pattern_follower/application.hpp>

Application::Application(const cv::FileStorage &fs)
    : camera_(new Camera(fs["Camera"])),
      robotControl_(
          new RobotControl(fs["RobotControl"], (int)fs["simulation"])) {}

Application::~Application() {
  cameraThread_.join();
  robotControlThread_.join();
}

void Application::run() {
  cameraThread_ = std::thread(&Application::cam_proc, this);
  robotControlThread_ = std::thread(&Application::rc_proc, this);
}

void Application::cameraThreadProcess() {
  double dist{0.0}, angle{0.0};
  bool suceed{false};
  while (true) {
    suceed = camera->proceed(angle, dist);

    g_i_mutex.lock();
    setGlobalVariables(dist, angle, suceed);
    g_i_mutex.unlock();

    if (waitKey(50) >= 0)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void Application::robotControlThreadProcess() {
  double dist{0.0}, angle{0.0};
  bool suceed{false};
  while (true) {
    g_i_mutex.lock();
    getGlobalVariable(dist, angle, suceed);
    g_i_mutex.unlock();
    rc->calculateRobotSpeeds(std::vector<cv::Point2d>(),
                             cv::Point2d(dist, angle), suceed);

    if (waitKey(50) >= 0)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
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
