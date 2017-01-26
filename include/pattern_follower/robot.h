#ifndef ROBOT_H
#define ROBOT_H
#include <chrono>
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
#define DIST_EPS 5.0

#define ANGLE_KP 3.0
#define ANGLE_KI 0.0
#define ANGLE_KD 1.0
#define ANGLE_EPS 0.15

class Robot {
public:
  Robot(const double &_maxVel, const double &_maxAngVel, const double &_a);

  Robot(const double &_maxVel, const double &_maxAngVel, const double &_a,
        char *IP, int &port);

  static void data_callback(CPositionMessage *pos) { printf("mes"); }

  virtual ~Robot() {
    rcm.close();
    if (client)
      client->sendControl(0, 0);
  };

  const double &getMaxVel() const { return maxVel; }

  const double &getMaxAngVel() const { return maxAngVel; }

  const double &getX() const { return x; }

  const double &getY() const { return y; }

  const double &getH() const { return h; }

  void computeH(const double &dt) {
    h += angVel * dt;
    normalizeAngle(h);
  }

  const double &getVel() const { return vel; }

  const double &getAngVel() const { return angVel; }

  void computePos(const double &dt) {
    x += (vel * cos(h) * dt);
    y += (vel * sin(h) * dt);
  }

  void normalizeAngle(double &angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
  }

  void setVel(const double &_vel);

  void setAngVel(const double &_angVel);

  bool move(const double &angle, const double &distance);

  void move_simulation(cv::Mat &frame, const double &angle,
                       const double &distance);

protected:
private:
  double maxVel, maxAngVel, vel, angVel, h, x, y, a, angVelDead, velDead;
  PID angularVelControl, velControl;
  RCM rcm;
  std::unique_ptr<CPositionClient> client;
  std::unique_ptr<CMessageClient> message_client;
  std::chrono::steady_clock::time_point lastUpdate;
  bool initRcm();
  bool enableMotion();
  void drawRobot(cv::Mat &frame, const double &angle, const double &dist);
  void calculateSpeeds(const double &angle, const double &distance);
};

#endif // ROBOT_H
