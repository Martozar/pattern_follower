#include <pattern_follower/pid.h>

double PID::calculate(const double & setPoint, const double & systemOutput, const double & dt)
{
    double error = setPoint - systemOutput;

    if(error >= - eps && error <= eps)
        return 0;

    if (error > eps)
        error -= eps;
    else if (error < -eps)
        error += eps;

    //std::cout << "Error: " << error <<std::endl;
    double Pout = Kp*error;

    double new_integral = integral + error*dt;

    double Iout = Ki*new_integral;

    //std::cout << "Int: " << integral << std::endl;

    double derivative = (error - previousError)/dt;

    double Dout = Kd*derivative;

    double output = Pout + Iout + Dout;

    privPresatOut = output;

    //std::cout << "Vystup: " << output << std::endl;
    if(output > max_out){
        output = max_out;
        integral += error*dt*Ka;
    }
    else if(output < min_out){
        output = min_out;
        integral += error*dt*Ka;
    }
    else
        integral = new_integral;



    privOut = output;
    previousError = error;
    return output;
}
