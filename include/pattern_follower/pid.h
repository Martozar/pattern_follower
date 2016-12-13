#ifndef PID_H
#define PID_H

#include <iostream>
#include <fstream>

class PID
{
    public:
        PID(const double & _Kp, const double &  _Ki, const double &  _Kd, const double &  _Ka, const double &  _max, const double &  _min, const double  & _eps) :
        Kp(_Kp), Ki(_Ki), Kd(_Kd), Ka(_Ka), max_out(_max), min_out(_min), previousError(0), integral(0)
        {
            privPresatOut = 0;
            privOut = 0;
            eps = _eps;
        };
        virtual ~PID() {};
        double calculate(const double & setPoint, const double & systemOutput, const double & dt);

    protected:

    private:
        double Kp, Ki, Kd, Ka, max_out, min_out, eps;
        double previousError, integral, privPresatOut, privOut;
};

#endif // PID_H
