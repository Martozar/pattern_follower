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

#define DIST_KP 5.0
#define DIST_KI 0.0
#define DIST_KD 1.0
#define DIST_EPS 0.1

#define ANGLE_KP 3.0
#define ANGLE_KI 0.0
#define ANGLE_KD 1.0
#define ANGLE_EPS 0.1

class Robot {
public:
  Robot();
  Robot(const cv::Point2d &setPoint, const double &maxVel, const double &radius,
        const double &acceleration);

  Robot(const cv::Point2d &setPoint, const double &maxVel, const double &radius,
        const double &acceleration, char *IP, int &port);

  static void data_callback(CPositionMessage *pos) { printf("mes"); }

  virtual ~Robot() {
    rcm_.close();
    if (client_)
      client_->sendControl(0, 0);
  };

  const double &getMaxVel() const { return maxVel_; }

  const double &getMaxAngVel() const { return maxAngVel_; }

  const double &getLeftSpeed() const { return velL_; }

  const double &getRightSpeed() const { return velR_; }

  const double &getX() const { return x_; }

  const double &getY() const { return y_; }

  const double &getH() const { return h_; }

  const double &getVel() const { return vel_; }

  const double &getAngVel() const { return angVel_; }

  void computeH(const double &dt) {
    h_ += (velR_ - velL_) / 0.2 * dt;
    normalizeAngle(h_);
  }

  void computePos(const double &dt) {
    x_ += (vel_ * std::cos(h_) * dt);
    y_ += (vel_ * std::sin(h_) * dt);
  }

  void normalizeAngle(double &angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
  }

  bool move(const double &angle, const double &distance,
            bool simulation = true);

  void move_simulation(cv::Mat &frame, const double &angle,
                       const double &distance);

protected:
private:
  double radius_, maxVel_, maxAngVel_, velL_, velR_, vel_, angVel_, h_, x_, y_,
      acceleration_, distance_, angle_;
  std::unique_ptr<PID> angularVelControl_, velControl_;
  RCM rcm_;
  std::unique_ptr<CPositionClient> client_;
  std::unique_ptr<CMessageClient> messageClient_;
  clock_t lastUpdate_;
  bool initRcm();
  bool enableMotion();
  void drawRobot(cv::Mat &frame, const double &angle, const double &dist);
  void calculateSpeeds(const double &angle, const double &distance);
  void setVel(const double &_vel);
  void setAngVel(const double &_angVel);
  void setWheelSpeeds(const double &linearVelocity,
                      const double &angularVelocity);
};

#endif // ROBOT_H
