#include <pattern_follower/robot.h>

Robot::Robot(const double &_maxVel, const double &_maxAngVel, const double &_a)
    : angularVelControl(ANGLE_KP, ANGLE_KI, ANGLE_KD, 1.0, _maxAngVel,
                        -_maxAngVel, ANGLE_EPS),
      velControl(DIST_KP, DIST_KI, DIST_KD, 0.0, _maxVel, -_maxVel, DIST_EPS) {
  maxVel = _maxVel;
  maxAngVel = _maxAngVel;
  a = _a;
  x = 0;
  y = 0;
  h = 0;
  vel = 0;
  angVel = 0;
  lastUpdate = std::chrono::steady_clock::now();
};

Robot::Robot(const double &_maxVel, const double &_maxAngVel, const double &_a,
             char *IP, int &port)
    : Robot(_maxVel, _maxAngVel, _a) {
  // client = std::make_unique<CPositionClient>(IP, port, data_callback);
  message_client = std::make_unique<CMessageClient>();
  bool p[2] = {true, true};
  message_client->init(IP, port, p);

  /*if (initRcm() && enableMotion())
    std::cout << "Suceed\n";*/
}

bool Robot::initRcm() {
  bool result = true;
  bool ret;
  printf("initRcm() ... ");
  // ret = rcm.Open("/dev/ttyS50");
  ret = rcm.Open("/dev/ttyUSB0");
  std::cout << "Opens: " << ret << std::endl;
  if (ret) {
    ret = rcm.InitDefaults();
    std::cout << "InitDef: " << ret << std::endl;
    if (ret) {
      // OK
    } else {

      rcm.close();
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
    result = rcm.setPower(2, 1.0, true);
    std::cout << "SetPow: " << result << std::endl;
    if (!result)
      break;
    result = rcm.setVelocities(0.0, 0.0, true);
    std::cout << "SetVel: " << result << std::endl;
    if (!result)
      break;
    // rcmPower = true;
    // result = readOdometry();
  } while (false);
  return result;
}

void Robot::setVel(const double &_vel) {
  if (vel < _vel) {
    vel += a;
    if (vel > _vel)
      vel = _vel;
  } else if (vel > _vel) {
    vel -= 3 * a;
    if (vel < _vel)
      vel = _vel;
  }
  if (vel > maxVel)
    vel = maxVel;
  else if (vel < -maxVel)
    vel = -maxVel;
}

void Robot::setAngVel(const double &_angVel) {
  if (angVel > _angVel) {
    angVel += a * 0.1;
    if (angVel > _angVel)
      angVel = _angVel;
  } else if (angVel < _angVel) {
    angVel -= 3 * a * 0.1;
    if (angVel < _angVel)
      angVel = _angVel;
  }
  if (angVel > maxAngVel)
    angVel = maxAngVel;
  else if (angVel < -maxAngVel)
    angVel = -maxAngVel;
}

bool Robot::move(const double &angle, const double &distance) {
  CMessage message;
  message.type = MSG_SPEED;
  message.forward = this->vel;
  message.turn = this->angVel * 5;
  if (message.forward > -20 && message.forward < 20)
    message.turn = 0;
  message.flipper = 0;
  //  std::cout << "Forward: " << vel << "\nAng_vel " << angVel << std::endl;
  bool ret = message_client->sendMessage(message);
  /*bool ret = client->sendControl((this->vel * 1000 / maxVel),
                                 (this->angVel * 1000 / maxAngVel) / 2);*/

  return ret;
}
void Robot::move_simulation(cv::Mat &frame, const double &angle,
                            const double &distance) {

  double new_angle = angle;
  this->normalizeAngle(new_angle);
  this->calculateSpeeds(new_angle, distance);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff = end - lastUpdate;
  this->computeH(diff.count());
  this->computePos(diff.count());
  this->drawRobot(frame, new_angle, distance);
  lastUpdate = end;
}
void Robot::drawRobot(cv::Mat &frame, const double &angle, const double &dist) {
  cv::circle(frame, cv::Point2f(x, y), 50, cv::Scalar(0, 255, 255), 2);
  cv::line(frame, cv::Point2f(x, y),
           cv::Point2f(x + (50 * cos(h)), y + (50 * sin(h))),
           cv::Scalar(0, 255, 255), 2);
  cv::String s = "Rychlost: ";
  s += std::to_string(vel);
  cv::putText(frame, s, cv::Point2f(450, 100), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
  s = "Uh. rychlost: " + std::to_string(angVel);
  cv::putText(frame, s, cv::Point2f(430, 115), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
  s = "Head: " + std::to_string(h);
  cv::putText(frame, s, cv::Point2f(430, 130), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
  s = "Vzdalenost: " + std::to_string(dist);
  cv::putText(frame, s, cv::Point2f(450, 145), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
  s = "Uhel: " + std::to_string(angle);
  cv::putText(frame, s, cv::Point2f(450, 160), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 0, 255));
}

void Robot::calculateSpeeds(const double &angle, const double &distance) {
  // std::cout << "angle: " << angle << "\ndistance: " << distance << std::endl;
  double angVel = angularVelControl.calculate(0.0, angle);
  double vel = velControl.calculate(-80.0, -distance);
  setAngVel(angVel);
  setVel(vel);
}
