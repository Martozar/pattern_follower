#include <pattern_follower/robot.h>

bool Robot::initRcm()
{
    bool result = true;
    bool ret;
    printf("initRcm() ... ");
    //ret = rcm.Open("/dev/ttyS50");
    ret = rcm.Open("/dev/ttyUSB0");
    std::cout << "Opens: "<<ret <<std::endl;
    if (ret)
    {
        ret = rcm.InitDefaults();
        std::cout << "InitDef: " << ret <<std::endl;
        if (ret)
        {
            // OK
        }
        else
        {

            rcm.close();
            result = false;
        }
    }
    else
    {
        result = false;
    }
    printf("done\n");
    return result;
}

bool Robot::enableMotion()
{
    bool result;
    do
    {
        result = rcm.setPower(2, 1.0, true);
        std::cout << "SetPow: "<<result <<std::endl;
        if (!result) break;
        result = rcm.setVelocities(0.0, 0.0, true);
        std::cout << "SetVel: " <<result <<std::endl;
        if (!result) break;
        //rcmPower = true;
        //result = readOdometry();
    }
    while(false);
    return result;
}


void Robot::setVel(const double & _vel)
{
    if(vel < _vel)
        vel+= a;
    else if(vel > _vel)
        vel-=3*a;
    if(vel > maxVel)
        vel = maxVel;
    else if(vel < -maxVel)
        vel = -maxVel;
}

void Robot::setAngVel(const double & _angVel)
{
    if(angVel > _angVel)
        angVel+= a*0.1;
    else if(angVel < _angVel)
        angVel-=3*a*0.1;
    if(angVel > maxAngVel)
        angVel = maxAngVel;
    else if(angVel < -maxAngVel)
        angVel = -maxAngVel;
}


bool Robot::move(double & vel, double & angVel)
{
    /*double sL = (vel/maxVel - angVel/maxAngVel);
    double sR = (vel/maxVel + angVel/maxAngVel);
    if(sL > 1) sL = 1;
    else if (sL < -1) sL = -1;

    if(sR > 1) sR = 1;
    else if (sR < -1) sR = -1;
    rcm.setVelocities(sR, -sL, true);*/
    setAngVel(-angVel);
    setVel(vel);



    return client->sendControl((this->vel*1000/maxVel), (this->angVel*1000/maxAngVel)/2);
}

    void Robot::drawRobot(cv::Mat & frame, const double & dist, const double & angle)
    {
        cv::circle(frame, cv::Point2f(x,y), 50, cv::Scalar(0,255,255), 2);
        cv::line(frame, cv::Point2f(x,y), cv::Point2f(x+(50*cos(h)),y+(50*sin(h))), cv::Scalar(0,255,255),2);
        cv::String s = "Rychlost: ";
        s += std::to_string(vel);
        cv::putText(frame, s, cv::Point2f(450, 100), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,255));
        s = "Uh. rychlost: " + std::to_string(angVel);
        cv::putText(frame, s, cv::Point2f(430, 115), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,255));
        s = "Head: " + std::to_string(h);
        cv::putText(frame, s, cv::Point2f(430, 130), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,255));
        s = "Vzdalenost: " + std::to_string(dist);
        cv::putText(frame, s, cv::Point2f(450, 145), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,255));
        s = "Uhel: " + std::to_string(angle);
        cv::putText(frame, s, cv::Point2f(450, 160), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,255));

    }
