#ifndef ROBOT_H
#define ROBOT_H

#include <ctime>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pattern_follower/CMessage.h>
#include <pattern_follower/CMessageClient.h>
#include <pattern_follower/camera.h>
#include <pattern_follower/pid.h>
#include <pattern_follower/pos_client.h>
#include <pattern_follower/rcm.h>
#include <string>

const double DIST_KP = 5.0;
const double DIST_KI = 0.0;
const double DIST_KD = 1.0;
const double DIST_EPS = 0.1;

const double ANGLE_KP = 3.0;
const double ANGLE_KI = 0.0;
const double ANGLE_KD = 1.0;
const double ANGLE_EPS = 0.035;

const double ACCELERATION = 0.1;
const double MAX_SPEED = 2.5;
const double RADIUS = 0.1;
const double WHEEL_RADIUS = 0.04;

class Robot {
public:
  Robot();
  Robot(const cv::Point2d &setPoint, bool simulation = true);

  static void data_callback(CPositionMessage *pos) { printf("mes"); }

  virtual ~Robot() {
    rcm_.close();
    if (client_)
      client_->sendControl(0, 0);
  };

  void setMaxVel(const double &ratio) { maxVel_ = MAX_SPEED * ratio; }
  const double &getMaxVel() const { return maxVel_; }

  const double &getMaxAngVel() const { return maxAngVel_; }

  const double &getLeftSpeed() const { return velL_; }

  const double &getRightSpeed() const { return velR_; }

  const double &getX() const { return x_; }

  const double &getY() const { return y_; }

  const double &getH() const { return h_; }

  const double &getVel() const { return vel_; }

  const double &getAngVel() const { return angVel_; }

  void updatePosition(const double &posL, const double &posR);

  void normalizeAngle(double &angle);

  bool move(const double &angle, const double &distance);

  void move_simulation(cv::Mat &frame, const double &angle,
                       const double &distance);

  double prevPosLeft_{0.0}, prevPosRight_{0.0};

protected:
private:
  double radius_, maxVel_, maxAngVel_, acceleration_, distance_, angle_,
      wheelRad_, velL_{0.0}, velR_{0.0}, vel_{0.0}, angVel_{0.0}, h_{0.0},
      x_{0.0}, y_{0.0};
  bool simulation_;
  std::unique_ptr<PID> angularVelControl_, velControl_;
  RCM rcm_;
  std::unique_ptr<CPositionClient> client_;
  std::unique_ptr<CMessageClient> messageClient_;
  clock_t lastUpdate_;
  bool initRcm();
  bool enableMotion();
  void drawRobot(cv::Mat &frame, const double &angle, const double &dist);
  void calculateSpeeds(const double &angle, const double &distance,
                       const double &dt);
  void setVel(const double &_vel, const double &dt);
  void setAngVel(const double &_angVel, const double &dt);
  void setWheelSpeeds(const double &linearVelocity,
                      const double &angularVelocity);
};

#endif // ROBOT_H
