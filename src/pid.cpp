#include <chrono>
#include <pattern_follower/pid.h>

double PID::calculate(const double &setPoint, const double &systemOutput) {
  double error = setPoint - systemOutput;

  auto now = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = now - lastUpdate;
  double dt = diff.count();
  lastUpdate = now;

  if (error >= -eps && error <= eps)
    return 0;

  if (error > eps)
    error -= eps;
  else if (error < -eps)
    error += eps;

  double Pout = Kp * error;

  double new_integral = integral + error * dt;

  double Iout = Ki * new_integral;

  double derivative = (error - previousError) / dt;

  double Dout = Kd * derivative;

  double output = Pout + Iout + Dout;

  privPresatOut = output;
  if (output > max_out) {
    output = max_out;
    integral += error * dt * Ka;
  } else if (output < min_out) {
    output = min_out;
    integral += error * dt * Ka;
  } else
    integral = new_integral;

  privOut = output;
  previousError = error;

  return output;
}
