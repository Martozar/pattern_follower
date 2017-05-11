#include <pattern_follower/application.h>

Application::Application(const std::string &path) {

  cv::FileStorage fs{path, FileStorage::READ};
  camera_ = std::unique_ptr<Camera>(new Camera(fs["Camera"]));
  robotControl_ = std::unique_ptr<RobotControl>(
      new RobotControl(fs["RobotControl"], (int)fs["simulation"]));
  isLaser = (int)fs["laser"];
}

Application::~Application() {
  cameraThread_.join();
  robotControlThread_.join();
  if (isLaser) {
    dataThread_.join();
    laser.close();
  }
}

void Application::run() {
  XInitThreads();
  if (isLaser) {
    laser.open("type=serial,device=/dev/ttyACM0,timeout=1");
    laser.set_power(true);
    laser.set_motor_speed(0);
    laser.set_multiecho_mode(hokuyoaist::ME_OFF);
    dataThread_ = std::thread(&Application::dataReadThreadProcess, this);
  }
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
    std::cout << "Camera\n";
    mutex_.unlock();
  }
}

void Application::dataReadThreadProcess() {
  while (!done_) {
    std::cout << "Laser\n";
    hokuyoaist::ScanData data;
    laser.get_new_ranges_by_angle(data, FIRST, LAST, 1);
    mutex_.lock();
    for (int i = 0; i < data.ranges_length(); i++) {
      double angle = FIRST + i * STEP;
      obstacles_.push_back(cv::Point2d(data[i] / 10.0 * std::cos(angle),
                                       data[i] / 10.0 * std::sin(angle)));

      // std::cout << obstacles_.back() << "\n";
    }
    if (waitKey(2) >= 0)
      done_ = true;
    mutex_.unlock();
  }
}

void Application::robotControlThreadProcess() {
  double dist{0.0}, angle{0.0};
  bool suceed{false};
  std::vector<cv::Point2d> obstacle;
  while (!done_) {

    mutex_.lock();

    std::cout << "Rob\n";
    getGlobalVariable(dist, angle, suceed);
    obstacle = obstacles_;

    if (waitKey(20) >= 0)
      done_ = true;

    mutex_.unlock();
    robotControl_->calculateRobotSpeeds(obstacle, cv::Point2d(dist, angle),
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
