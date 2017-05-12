#include <pattern_follower/application.h>

Application::Application(const std::string &path) {

  cv::FileStorage fs{path, FileStorage::READ};
  camera_ = std::unique_ptr<Camera>(new Camera(fs["Camera"]));
  robotControl_ = std::unique_ptr<RobotControl>(
      new RobotControl(fs["RobotControl"], (int)fs["simulation"]));
  isLaser = fs["laser"];
}

Application::~Application() {
  cameraThread_.join();
  robotControlThread_.join();
  robotStatus_.join();
#ifdef WITH_LASER
  if (isLaser) {
    dataThread_.join();
    laser.close();
  }
#endif
}

void Application::run() {
  XInitThreads();
#ifdef WITH_LASER
  if (isLaser) {
    laser.open("type=serial,device=/dev/ttyACM0,timeout=1");
    laser.set_power(true);
    laser.set_motor_speed(0);
    laser.set_multiecho_mode(hokuyoaist::ME_OFF);
    dataThread_ = std::thread(&Application::dataReadThreadProcess, this);
  }
#endif
  cameraThread_ = std::thread(&Application::cameraThreadProcess, this);
  robotControlThread_ =
      std::thread(&Application::robotControlThreadProcess, this);
  robotStatus_ = std::thread(&Application::robotStatusControl, this);

  while (!done_)
    if (waitKey(20) >= 0)
      done_ = true;
}

void Application::robotStatusControl() {
  while (!done_) {
    robMutex_.lock();
    robotControl_->getRobot()->checkStatus();
    robMutex_.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void Application::cameraThreadProcess() {
  double dist{0.0}, angle{0.0};
  bool suceed{false};
  while (!done_) {
    suceed = camera_->proceed(angle, dist);

    camMutex_.lock();
    // std::cout << "Camera\n";
    setGlobalVariables(dist, angle, suceed);
    camMutex_.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void Application::dataReadThreadProcess() {
#ifdef WITH_LASER
  while (!done_) {
    hokuyoaist::ScanData data;
    laser.get_new_ranges_by_angle(data, FIRST, LAST, 1);
    laserMutex_.lock();

    obstacles_.clear();
    // std::cout << "Laser\n";
    for (int i = 0; i < data.ranges_length(); i++) {
      double angle = FIRST + i * STEP;
      if (data[i] > 50)
        obstacles_.push_back(cv::Point2d(data[i] / 10.0 * std::cos(angle),
                                         data[i] / 10.0 * std::sin(angle)));
      // std::cout << obstacles_.back() << "\n";
    }
    laserMutex_.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
#endif
}

void Application::robotControlThreadProcess() {
  double dist{0.0}, angle{0.0};
  bool suceed{false};
  std::vector<cv::Point2d> obstacle;
  while (!done_) {

    camMutex_.lock();
    // std::cout << "Rob\n";
    getGlobalVariable(dist, angle, suceed);
    camMutex_.unlock();

    laserMutex_.lock();
    obstacle = obstacles_;
    laserMutex_.unlock();

    robMutex_.lock();
    robotControl_->calculateRobotSpeeds(obstacle, cv::Point2d(dist, angle),
                                        suceed);
    robMutex_.unlock();

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
