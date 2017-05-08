#include <chrono>
#include <pattern_follower/pid.h>

PID::PID(const cv::FileNode &fn, const double &max, const double &min) {
  Kp_ = fn["kp"];
  Ki_ = fn["ki"];
  Kd_ = fn["kd"];
  Ka_ = fn["ka"];
  eps_ = fn["eps"];
  maxOut_ = max;
  minOut_ = min;
  lastUpdate_ = std::chrono::steady_clock::now();
}

double PID::calculate(const double &setPoint, const double &systemOutput) {
  double error = -setPoint + systemOutput;

  auto now = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = now - lastUpdate_;
  double dt = diff.count();
  lastUpdate_ = now;

  if (error >= -eps_ && error <= eps_)
    return 0;

  if (error > eps_)
    error -= eps_;
  else if (error < -eps_)
    error += eps_;

  double Pout = Kp_ * error;

  double new_integral = integral_ + error * dt;

  double Iout = Ki_ * new_integral;

  double derivative = (error - previousError_) / dt;

  double Dout = Kd_ * derivative;

  double output = Pout + Iout + Dout;

  privPresatOut_ = output;
  if (output > maxOut_) {
    output = maxOut_;
    integral_ += error * dt * Ka_;
  } else if (output < minOut_) {
    output = minOut_;
    integral_ += error * dt * Ka_;
  } else
    integral_ = new_integral;

  privOut_ = output;
  previousError_ = error;

  return output;
}
