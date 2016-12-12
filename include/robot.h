#ifndef ROBOT_H
#define ROBOT_H
#include <math.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "rcm.h"
#include "pos_client.h"


class Robot
{
public:
    Robot(const double & _maxVel, const double & _maxAngVel, const double & _a, const double & _angVelDead, const double & _velDead)
    {
        maxVel = _maxVel;
        maxAngVel = _maxAngVel;
        angVelDead = _angVelDead;
        velDead = _velDead;
        a = _a;
        x = 0;
        y = 0;
        h = 0;
        vel = 0;
        angVel = 0;
    };

    Robot(const double & _maxVel, const double & _maxAngVel, const double & _a, const double & _angVelDead, const double & _velDead, char * IP, int & port) :
    Robot(_maxVel, _maxAngVel, _a, _angVelDead, _velDead)
    {
        client = std::make_unique<CPositionClient>(IP, port, data_callback);

        if(initRcm() && enableMotion())
            std::cout << "Suceed\n";
    }

    static void data_callback(CPositionMessage* pos)
    {
        printf("mes");
    }

    virtual ~Robot()
    {
        rcm.close();
        if(client) client->sendControl(0,0);
    };

    const double &getMaxVel() const
    {
        return maxVel;
    }

    const double &getMaxAngVel() const
    {
        return maxAngVel;
    }

    const double & getX() const
    {
        return x;
    }

    const double & getY() const
    {
        return y;
    }

    const double & getH() const
    {
        return h;
    }

    void computeH(const double & dt)
    {
        h += angVel*dt;
        normalizeAngle(h);
    }


    const double & getVel() const
    {
        return vel;
    }

    const double & getAngVel() const
    {
        return angVel;
    }

    void computePos(const double & dt)
    {
        x += (vel*cos(h)*dt);
        y += (vel*sin(h)*dt);
    }

    void normalizeAngle(double & angle)
    {
        while(angle > M_PI) angle -= 2*M_PI;
        while(angle < -M_PI) angle += 2*M_PI;
    }

    void setVel(const double & _vel);

    void setAngVel(const double & _angVel);

    void drawRobot(cv::Mat & frame, const double & dist, const double & angle);

    bool move(double & vel, double & angVel);

protected:

private:
    double maxVel, maxAngVel, vel, angVel, h, x, y, a, angVelDead, velDead;

    RCM rcm;
    std::unique_ptr<CPositionClient> client;
    bool initRcm();
    bool enableMotion();


};

#endif // ROBOT_H
