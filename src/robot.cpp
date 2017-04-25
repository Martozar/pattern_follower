#include <pattern_follower/robot.h>

Robot::Robot(const FileNode &fn, const bool &simulation) {
  wheelRad_ = fn["wheel_radius"];
  radius_ = fn["radius"];
  maxVel_ = fn["max_forward_speed"];
  maxAngVel_ = fn["max_angular_speed"];
  acceleration_ = fn["acceleration"];
  distance_ = fn["set_point_distance"];
  angle_ = fn["set_point_angle"];
  angularVelControl_ =
      std::unique_ptr<PID>(new PID(fn["Angle_PID"], maxAngVel_, -maxAngVel_));
  velControl_ =
      std::unique_ptr<PID>(new PID(fn["Distance_PID"], maxVel_, -maxVel_));
  lastUpdate_ = clock();
  if (!simulation) {
    messageClient_ = std::make_unique<CMessageClient>();
    bool p[2] = {true, true};
    messageClient_->init(((std::string)fn["IP"]).c_str(), fn["port"], p);
    // client = std::make_unique<CPositionClient>(IP, port, data_callback);

    /*if (initRcm() && enableMotion())
      std::cout << "Suceed\n";*/
  }
}

bool Robot::initRcm() {
  bool result = true;
  bool ret;
  printf("initRcm() ... ");
  // ret = rcm.Open("/dev/ttyS50");
  ret = rcm_.Open("/dev/ttyUSB0");
  std::cout << "Opens: " << ret << std::endl;
  if (ret) {
    ret = rcm_.InitDefaults();
    std::cout << "InitDef: " << ret << std::endl;
    if (ret) {
      // OK
    } else {

      rcm_.close();
      result = false;
    }
  } else {
    result = false;
  }
  printf("done\n");
  return result;
}

bool Robot::enableMotion() {
  bool result;
  do {
    result = rcm_.setPower(2, 1.0, true);
    std::cout << "SetPow: " << result << std::endl;
    if (!result)
      break;
    result = rcm_.setVelocities(0.0, 0.0, true);
    std::cout << "SetVel: " << result << std::endl;
    if (!result)
      break;
    // rcmPower = true;
    // result = readOdometry();
  } while (false);
  return result;
}
void Robot::updatePosition(const double &posL, const double &posR) {
  double dL = (posL - prevPosLeft_) * wheelRad_;
  double dR = (posR - prevPosRight_) * wheelRad_;
  prevPosLeft_ = posL;
  prevPosRight_ = posR;
  double dDist = (dL + dR) / 2.0;
  double dTheta = (dR - dL) / (radius_ * 2.0);
  std::cout << "Dr " << dR << " dl " << dL << " dth " << dTheta << "\n";
  double dx = dDist * std::cos(h_);
  double dy = dDist * std::sin(h_);
  h_ += dTheta;
  normalizeAngle(h_);
  x_ += dx;
  y_ += dy;
}

void Robot::setVel(const double &vel, const double &dt) {
  if (vel_ < vel) {
    vel_ += acceleration_ * dt;
    if (vel_ > vel)
      vel_ = vel;
  } else if (vel_ > vel) {
    vel_ -= 3.0 * acceleration_ * dt;
    if (vel_ < vel)
      vel_ = vel;
  }
  vel_ = vel;
  if (vel_ > maxVel_)
    vel_ = maxVel_;
  else if (vel_ < -maxVel_)
    vel_ = -maxVel_;
}

void Robot::setAngVel(const double &angVel, const double &dt) {
  if (angVel_ < angVel) {
    angVel_ += acceleration_ * dt;
    if (angVel_ > angVel)
      angVel_ = angVel;
  } else if (vel_ > angVel) {
    angVel_ -= 3.0 * acceleration_ * dt;
    if (angVel_ < angVel)
      angVel_ = angVel;
  }
  if (angVel_ > maxAngVel_)
    angVel_ = maxAngVel_;
  else if (vel_ < -maxAngVel_)
    angVel_ = -maxAngVel_;
}

bool Robot::move(const double &angle, const double &distance) {
  bool ret = true;

  double new_angle = angle;
  this->normalizeAngle(new_angle);
  clock_t end = clock();
  double elapsed_secs = double(end - lastUpdate_) / CLOCKS_PER_SEC;
  calculateSpeeds(new_angle, distance, elapsed_secs);
  lastUpdate_ = end;
  if (!simulation_) {
    CMessage message;
    message.type = MSG_SPEED;
    message.forward = this->vel_;
    message.turn = this->angVel_ * 5;
    if (message.forward > -20 && message.forward < 20)
      message.turn = 0;
    message.flipper = 0;
    ret = messageClient_->sendMessage(message);
  }
  return ret;
}

void Robot::calculateSpeeds(const double &angle, const double &distance,
                            const double &dt) {
  double angVel = angularVelControl_->calculate(angle_, angle);
  double vel = velControl_->calculate(-distance_, -distance);
  setVel(vel, dt);
  setAngVel(angVel, dt);
  setWheelSpeeds(vel, angVel);
}

void Robot::setWheelSpeeds(const double &linearVelocity,
                           const double &angularVelocity) {
  velR_ = vel_ + angVel_;
  velL_ = vel_ - angVel_;
}

void Robot::move_simulation(cv::Mat &frame, const double &angle,
                            const double &distance) {

  double new_angle = angle;
  this->normalizeAngle(new_angle);
  clock_t end = clock();

  double elapsed_secs = double(end - lastUpdate_) / CLOCKS_PER_SEC;

  this->calculateSpeeds(new_angle, distance, elapsed_secs);
  this->drawRobot(frame, new_angle, distance);
  lastUpdate_ = end;
}

void Robot::normalizeAngle(double &angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
}

void Robot::drawRobot(cv::Mat &frame, const double &angle, const double &dist) {
  cv::circle(frame, cv::Point2f(x_, y_), 50, cv::Scalar(0, 255, 255), 2);
  cv::line(frame, cv::Point2f(x_, y_),
           cv::Point2f(x_ + (50 * std::cos(h_)), y_ + (50 * std::sin(h_))),
           cv::Scalar(0, 255, 255), 2);
  cv::String s = "Rychlost: ";
  s += std::to_string(vel_);
  cv::putText(frame, s, cv::Point2f(450, 100), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
  s = "Uh. rychlost: " + std::to_string(angVel_);
  cv::putText(frame, s, cv::Point2f(430, 115), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
  s = "Head: " + std::to_string(h_);
  cv::putText(frame, s, cv::Point2f(430, 130), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
  s = "Vzdalenost: " + std::to_string(dist);
  cv::putText(frame, s, cv::Point2f(450, 145), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
  s = "Uhel: " + std::to_string(angle);
  cv::putText(frame, s, cv::Point2f(450, 160), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
}
