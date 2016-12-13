/*
   File name: er1robot.h
   Date:      2005/06/22 09:15
   Author:    
*/


#ifndef __ER1ROBOT_H__
#define __ER1ROBOT_H__

#include <pthread.h>
#include <unistd.h>
#include "rcm.h"

class CPosition
{
public:
	double x,y,h;
};

typedef enum{
	taskNONE, taskSTOP, taskWAIT, taskWAITEVENT, taskGO,
	taskLINE, taskCURVE, taskTURN, taskTURNTO,
	taskPOINT, taskPOSITION,
	taskTERMINATE,
	taskNUMBER
}TTaskType;

class CTask
{
public:
	TTaskType type;
	bool initialized;
	int duration;
	long finalTime;
	double speed;
	double steeringSpeed;
	double length;
	double angle;
	double finalAngle;
	double radius;
	CPosition destination;
	bool destinationHeadingValid;
	int eventCode;
	double tresholdLine[3];
};

class CER1Robot
{
private:
	pthread_mutex_t er1_lock;
	bool terminate;
	bool terminated;

	double x, y, h;
	int controlPeriod;

	int pathLength;
	int pathSize;
	CTask* path;
	int pathPointer;
	bool motionEnabled;
	bool velocityControlMode;

	double lastOdo[2];
	bool lastOdoValid;

	RCM rcm;
	bool rcmPower;

	int maxEventCount;
	int* eventList;

	double leftWheelVelocity;
	double rightWheelVelocity;
	long lastVelocityCommandTime;
public:
	CER1Robot();
	~CER1Robot();
	int run();
	void interrupt(){terminate = true;};
	bool isInterrupted(){return terminated;};
	
	bool allocatePath(int length);
	void clearPath();
	bool addPathPoint(CPosition* point, bool headingValid);
	bool addStopCommand();
	bool addGoCommand(double speed, double steering, int time);
	bool addWaitCommand(int time);
	bool addWaitEvent(int eventCode, int timeout);
	bool addLine(double length, double speed);
	bool addCurve(double angle, double radius, double speed);
	bool addTurn(double angle, double speed);
	bool addTurnTo(double heading, double speed);
	bool addTerminateCommand();
	
	void go();
	void stop();
	void setRobotPosition(double iX, double iY, double iH);
	void getRobotPosition(double &x, double &y, double &h);

	bool setRelativeVelocities(double forwardVelocity, double angularVelocity);

	bool resetEvents(int count);
	bool externalEvent(int code);
	bool isEvent(int code);
private:
	bool processTask(CTask* task);
	bool moveToDestination(CPosition* destination);

	bool stopRobot();
	bool moveRobot(CTask* task);
	bool wait(CTask* task);
	bool moveLine(CTask* task);
	bool moveCurve(CTask* task);
	bool moveTurnTo(CTask* task);
	bool moveTurnBy(CTask* task);

	void processVelocityControlMode();

	bool initRcm();
	bool closeRcm();

	bool enableMotion();
	bool disableMotion();

	bool readOdometry();
};

#endif

/* end of er1robot.h */
