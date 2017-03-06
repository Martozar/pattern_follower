#include <pattern_follower/robot.h>

Robot::Robot() {
  x_ = 0;
  y_ = 0;
  h_ = 0;
  velL_ = 0;
  velR_ = 0;
  vel_ = 0;
  angVel_ = 0;
  lastUpdate_ = clock();
}

Robot::Robot(const cv::Point2d &setPoint, const double &maxVel,
             const double &radius, const double &acceleration)
    : Robot() {
  distance_ = setPoint.x;
  angle_ = setPoint.y;
  radius_ = radius;
  maxVel_ = maxVel;
  maxAngVel_ = maxVel_;
  acceleration_ = acceleration;
  angularVelControl_ = std::unique_ptr<PID>(new PID(
      ANGLE_KP, ANGLE_KI, ANGLE_KD, 1.0, maxAngVel_, -maxAngVel_, ANGLE_EPS));
  velControl_ = std::unique_ptr<PID>(
      new PID(DIST_KP, DIST_KI, DIST_KD, 0.0, maxVel_, -maxVel_, DIST_EPS));
};

Robot::Robot(const cv::Point2d &setPoint, const double &maxVel,
             const double &radius, const double &acceleration, char *IP,
             int &port)
    : Robot(setPoint, maxVel, radius, acceleration) {
  // client = std::make_unique<CPositionClient>(IP, port, data_callback);
  messageClient_ = std::make_unique<CMessageClient>();
  bool p[2] = {true, true};
  messageClient_->init(IP, port, p);

  /*if (initRcm() && enableMotion())
    std::cout << "Suceed\n";*/
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

void Robot::setVel(const double &vel) {
  if (vel_ < vel) {
    vel_ += acceleration_;
    if (vel_ > vel)
      vel_ = vel;
  } else if (vel_ > vel) {
    vel_ -= 3 * acceleration_;
    if (vel_ < vel)
      vel_ = vel;
  }
  vel_ = vel;
  if (vel_ > maxVel_)
    vel_ = maxVel_;
  else if (vel_ < -maxVel_)
    vel_ = -maxVel_;
}

void Robot::setAngVel(const double &angVel) {
  if (angVel_ < angVel) {
    angVel_ += acceleration_;
    if (angVel_ > angVel)
      angVel_ = angVel;
  } else if (vel_ > angVel) {
    angVel_ -= 3 * acceleration_;
    if (angVel_ < angVel)
      angVel_ = angVel;
  }
  if (angVel_ > maxAngVel_)
    angVel_ = maxAngVel_;
  else if (vel_ < -maxAngVel_)
    angVel_ = -maxAngVel_;
}

bool Robot::move(const double &angle, const double &distance, bool simulation) {
  bool ret = true;

  double new_angle = angle;
  this->normalizeAngle(new_angle);
  std::cout << "NA " << new_angle << "\n";
  calculateSpeeds(new_angle, distance);
  clock_t end = clock();

  double elapsed_secs = double(end - lastUpdate_) / CLOCKS_PER_SEC;
  computeH(elapsed_secs);
  lastUpdate_ = end;
  if (!simulation) {
    CMessage message;
    message.type = MSG_SPEED;
    message.forward = this->vel_;
    message.turn = this->angVel_ * 5;
    if (message.forward > -20 && message.forward < 20)
      message.turn = 0;
    message.flipper = 0;
    //  std::cout << "Forward: " << vel << "\nAng_vel " << angVel << std::endl;
    ret = messageClient_->sendMessage(message);
    /*bool ret = client->sendControl((this->vel * 1000 / maxVel),
                                   (this->angVel * 1000 / maxAngVel) / 2);*/
  }
  return ret;
}

void Robot::calculateSpeeds(const double &angle, const double &distance) {
  double angVel = angularVelControl_->calculate(angle_, angle);
  double vel = velControl_->calculate(-distance_, -distance);
  std::cout << "AngVel " << angVel << " vel " << vel << "\n";
  setWheelSpeeds(vel, angVel);
}

void Robot::setWheelSpeeds(const double &linearVelocity,
                           const double &angularVelocity) {
  setVel(linearVelocity);
  setAngVel(angularVelocity);
  velR_ = vel_ + angVel_;
  velL_ = vel_ - angVel_;

  std::cout << "R " << velR_ << " L " << velL_ << "\n";
}
void Robot::move_simulation(cv::Mat &frame, const double &angle,
                            const double &distance) {

  double new_angle = angle;
  this->normalizeAngle(new_angle);
  this->calculateSpeeds(new_angle, distance);
  clock_t end = clock();

  double elapsed_secs = double(end - lastUpdate_) / CLOCKS_PER_SEC;
  this->computeH(elapsed_secs);
  this->computePos(elapsed_secs);
  this->drawRobot(frame, new_angle, distance);
  lastUpdate_ = end;
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
