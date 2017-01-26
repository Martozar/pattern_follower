#include <math.h>
#include <pattern_follower/er1robot.h>
#include <stdio.h>
#include <sys/time.h>

#define ER1_STEPS_PER_METER 172000.0
#define ER1_WHEEL_BASE 0.375

#define LOOP_SLEEP 100

#define GO_SPEED 0.3
#define TURN_SPEED 0.3
#define DESTINATION_TRESHOLD 0.02

//===================================================================

CER1Robot::CER1Robot() {
  printf("creating er1robot...\n");
  terminate = false;
  terminated = false;
  pthread_mutex_init(&er1_lock, NULL);

  x = 0.0;
  y = 0.0;
  h = 0.0;
  controlPeriod = 100;

  pathLength = 0;
  pathSize = 0;
  pathPointer = 0;
  motionEnabled = false;
  velocityControlMode = false;
  path = NULL;
  allocatePath(100);

  maxEventCount = 0;
  eventList = NULL;
  resetEvents(20);

  leftWheelVelocity = 0;
  rightWheelVelocity = 0;
  lastVelocityCommandTime = 0;

  lastOdoValid = false;

  rcmPower = false;
  initRcm();
  printf("er1robot created.\n");
}

CER1Robot::~CER1Robot() {
  terminate = true;
  usleep(1000 * 1000);
  closeRcm();
  delete[] path;
}

int CER1Robot::run() {
  terminated = false;
  while (1) {
    if (terminate)
      break;
    bool moving = false;
    pthread_mutex_lock(&er1_lock);
    if (motionEnabled) {
      if (!velocityControlMode) {
        if (path != NULL && pathLength > 0 && pathPointer >= 0 &&
            pathPointer < pathLength) {
          // printf("task %d\n", pathPointer);
          if (processTask(&path[pathPointer])) {
            printf("task %d done\n", pathPointer);
            pathPointer++;
            if (pathPointer == pathLength) {
              disableMotion();
            }
          }
        }
      } else {
        // printf("velocity control mode\n");
        processVelocityControlMode();
      }
      moving = true;
    } else {
      // printf("path=%p  len=%d  ptr=%d  mot=%d\n", path, pathLength,
      // pathPointer, motionEnabled);
    }
    pthread_mutex_unlock(&er1_lock);
    if (!moving) {
      usleep(1000 * LOOP_SLEEP);
    }
  }
  disableMotion();
  terminated = true;
}

bool CER1Robot::allocatePath(int size) {
  pthread_mutex_lock(&er1_lock);
  bool result = true;
  if (size > pathSize) {
    CTask *newPath = new CTask[size];
    if (newPath != NULL) {
      if (path != NULL) {
        for (int i = 0; i < pathLength; i++) {
          newPath[i] = path[i];
          // memcpy(&newPath[i], &path[i], sizeof(CTask));
        }
        delete path;
      }
      path = newPath;
      pathSize = size;
    } else {
      result = false;
    }
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

void CER1Robot::clearPath() {
  pthread_mutex_lock(&er1_lock);
  pathLength = 0;
  pathPointer = 0;
  pthread_mutex_unlock(&er1_lock);
}

bool CER1Robot::addPathPoint(CPosition *point, bool headingValid) {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    if (headingValid) {
      path[pathLength].type = taskPOSITION;
    } else {
      path[pathLength].type = taskPOINT;
    }
    path[pathLength].initialized = false;
    path[pathLength].destinationHeadingValid = headingValid;
    path[pathLength].destination = *point;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

bool CER1Robot::addStopCommand() {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    path[pathLength].type = taskSTOP;
    path[pathLength].initialized = false;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

bool CER1Robot::addGoCommand(double speed, double steering, int time) {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    path[pathLength].type = taskGO;
    path[pathLength].initialized = false;
    path[pathLength].speed = speed;
    path[pathLength].steeringSpeed = steering;
    path[pathLength].duration = time;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

bool CER1Robot::addWaitCommand(int time) {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    path[pathLength].type = taskWAIT;
    path[pathLength].initialized = false;
    path[pathLength].duration = time;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

bool CER1Robot::addWaitEvent(int eventCode, int timeout) {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    path[pathLength].type = taskWAITEVENT;
    path[pathLength].initialized = false;
    path[pathLength].duration = timeout;
    path[pathLength].eventCode = eventCode;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

bool CER1Robot::addLine(double length, double speed) {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    path[pathLength].type = taskLINE;
    path[pathLength].initialized = false;
    path[pathLength].length = length;
    path[pathLength].speed = speed;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

bool CER1Robot::addCurve(double angle, double radius, double speed) {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    path[pathLength].type = taskCURVE;
    path[pathLength].initialized = false;
    path[pathLength].angle = angle;
    path[pathLength].radius = radius;
    path[pathLength].speed = speed;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

bool CER1Robot::addTurn(double angle, double speed) {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    path[pathLength].type = taskTURN;
    path[pathLength].initialized = false;
    path[pathLength].angle = angle;
    path[pathLength].steeringSpeed = speed;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

bool CER1Robot::addTurnTo(double heading, double speed) {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    path[pathLength].type = taskTURNTO;
    path[pathLength].initialized = false;
    path[pathLength].angle = heading;
    path[pathLength].steeringSpeed = speed;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}

bool CER1Robot::addTerminateCommand() {
  bool result = true;
  pthread_mutex_lock(&er1_lock);
  if ((path != NULL) && (pathLength < pathSize)) {
    path[pathLength].type = taskTERMINATE;
    path[pathLength].initialized = false;
    pathLength++;
  } else {
    result = false;
  }
  pthread_mutex_unlock(&er1_lock);
  return result;
}
void CER1Robot::go() {
  pthread_mutex_lock(&er1_lock);
  motionEnabled = true;
  pthread_mutex_unlock(&er1_lock);
}

void CER1Robot::stop() {
  pthread_mutex_lock(&er1_lock);
  motionEnabled = false;
  pthread_mutex_unlock(&er1_lock);
}

void CER1Robot::setRobotPosition(double iX, double iY, double iH) {
  pthread_mutex_lock(&er1_lock);
  x = iX;
  y = iY;
  h = iH;
  pthread_mutex_unlock(&er1_lock);
}

void CER1Robot::getRobotPosition(double &x, double &y, double &h) {
  x = this->x;
  y = this->y;
  h = this->h;
}

bool CER1Robot::setRelativeVelocities(double forwardVelocity,
                                      double angularVelocity) {
  pthread_mutex_lock(&er1_lock);
  velocityControlMode = true;
  leftWheelVelocity = GO_SPEED * forwardVelocity - TURN_SPEED * angularVelocity;
  rightWheelVelocity =
      GO_SPEED * forwardVelocity + TURN_SPEED * angularVelocity;
  pthread_mutex_unlock(&er1_lock);
}

//===================================================================

bool CER1Robot::resetEvents(int count) {
  if (eventList)
    delete[] eventList;
  if (count == 0)
    eventList = NULL;
  else {
    eventList = new int[count];
    maxEventCount = count;
    for (int i = 0; i < count; i++) {
      eventList[i] = 0;
    }
  }
}

bool CER1Robot::externalEvent(int code) {
  bool result = true;
  if (code >= 0 && code < maxEventCount) {
    eventList[code] = 1;
  } else {
    result = false;
  }
  return result;
}

bool CER1Robot::isEvent(int code) {
  bool result = false;
  if (code >= 0 && code < maxEventCount) {
    result = (eventList[code] > 0);
  }
  return result;
}
//===================================================================
long getTime(void) {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  return (tp.tv_sec * 1000 + tp.tv_usec / 1000);
}
//===================================================================

bool CER1Robot::processTask(CTask *task) {
  bool result = true;
  switch (task->type) {
  case taskNONE:
    result = true;
    break;
  case taskPOSITION:
    // TODO headingValid
    result = moveToDestination(&task->destination);
    break;
  case taskPOINT:
    // TODO headingValid
    result = moveToDestination(&task->destination);
    break;
  case taskSTOP:
    result = stopRobot();
    break;
  case taskGO:
    result = moveRobot(task);
    break;
  case taskWAIT:
  case taskWAITEVENT:
    result = wait(task);
    break;
  case taskLINE:
    result = moveLine(task);
    break;
  case taskCURVE:
    result = moveCurve(task);
    break;
  case taskTURN:
    result = moveTurnBy(task);
    break;
  case taskTURNTO:
    result = moveTurnTo(task);
    break;
  case taskTERMINATE:
    terminate = true;
    result = true;
  default:
    printf("WARNING: unknown task %d\n", task->type);
    result = true;
    break;
  }
}

void normalizeAngle(double *a) {
  while (*a > M_PI)
    *a -= 2 * M_PI;
  while (*a < -M_PI)
    *a += 2 * M_PI;
}

double sign(double x) {
  if (x > 0.0)
    return 1.0;
  if (x < 0.0)
    return -1.0;
  return 0.0;
}

bool CER1Robot::moveToDestination(CPosition *destination) {
  bool reachedDestination = false;
  bool ret;
  ret = readOdometry();
  if (ret) {
    if (!rcmPower) {
      enableMotion();
    }

    double dx = destination->x - x;
    double dy = destination->y - y;
    double distance = sqrt(dx * dx + dy * dy);
    double direction = atan2(dy, dx);
    double dh = direction - h;
    normalizeAngle(&dh);

    double speedLeft = 0.0;
    double speedRight = 0.0;

    if (distance < DESTINATION_TRESHOLD) {
      // destination reached
      reachedDestination = true;
    } else {
      if (fabs(dh) > distance) {
        // turn to destination
        speedLeft = -TURN_SPEED * sign(dh);
        speedRight = TURN_SPEED * sign(dh);
      } else {
        // go to destination
        speedLeft = GO_SPEED - TURN_SPEED * distance * dh;
        speedRight = GO_SPEED + TURN_SPEED * distance * dh;
      }
    }

    rcm.setVelocities(speedRight, -speedLeft, true);
  }
}

bool CER1Robot::initRcm() {
  bool result = true;
  bool ret;
  printf("initRcm() ... ");
  // ret = rcm.Open("/dev/ttyS50");
  ret = rcm.Open("/dev/ttyUSB0");
  if (ret) {
    ret = rcm.InitDefaults();
    if (ret) {
      // OK
    } else {
      rcm.close();
      result = false;
    }
  } else {
    result = false;
  }
  printf("done\n");
  return result;
}

bool CER1Robot::closeRcm() {
  bool result = true;
  rcm.close();
  return result;
}

bool CER1Robot::enableMotion() {
  bool result;
  do {
    result = rcm.setPower(2, 1.0, true);
    if (!result)
      break;
    result = rcm.setVelocities(0.0, 0.0, true);
    if (!result)
      break;
    rcmPower = true;
    result = readOdometry();
  } while (false);
  return result;
}

bool CER1Robot::disableMotion() {
  bool result;
  do {
    result = rcm.setVelocities(0.0, 0.0, true);
    if (!result)
      break;
    result = rcm.setPower(2, 0.0, true);
    if (!result)
      break;
    rcmPower = false;
    result = readOdometry();
  } while (false);
  return result;
}

bool CER1Robot::readOdometry() {
  bool result = true;
  do {
    int iOdo[2];
    double odo[2];
    result = rcm.GetCommandedPositionSafe(0, iOdo[0]);
    if (!result)
      break;
    result = rcm.GetCommandedPositionSafe(1, iOdo[1]);
    if (!result)
      break;
    iOdo[1] = -iOdo[1];
    odo[0] = (double)iOdo[0] / ER1_STEPS_PER_METER;
    odo[1] = (double)iOdo[1] / ER1_STEPS_PER_METER;

    if (lastOdoValid) {
      double d0 = odo[0] - lastOdo[0];
      double d1 = odo[1] - lastOdo[1];
      double rot = atan2(d1 - d0, ER1_WHEEL_BASE);
      double dist = (d0 + d1) / 2.0;
      h = h - rot;
      normalizeAngle(&h);
      x = x + dist * cos(h);
      y = y + dist * sin(h);
    }

    lastOdo[0] = odo[0];
    lastOdo[1] = odo[1];
    lastOdoValid = true;
  } while (false);
}

bool CER1Robot::stopRobot() {
  rcm.setVelocities(0.0, 0.0, true);
  disableMotion();
  return true;
}

bool CER1Robot::moveRobot(CTask *task) {
  bool result = false;
  int t = getTime();
  if (!task->initialized) {
    task->finalTime = t + task->duration;
    double sL = task->speed - task->steeringSpeed;
    double sR = task->speed + task->steeringSpeed;
    if (sL > 1.0)
      sL = 1.0;
    if (sL < -1.0)
      sL = -1.0;
    if (sR > 1.0)
      sR = 1.0;
    if (sR < -1.0)
      sR = -1.0;
    if (!rcmPower) {
      enableMotion();
    }
    rcm.setVelocities(sR, -sL, true);
    task->initialized = true;
  }
  if (t > task->finalTime) {
    result = true;
  }
  return result;
}

bool CER1Robot::wait(CTask *task) {
  bool result = false;
  int t = getTime();
  if (!task->initialized) {
    stopRobot();
    printf("WAITING\n");
    task->finalTime = t + task->duration;
    task->initialized = true;
  }
  if (task->type == taskWAITEVENT) {
    if (task->duration > 0 && t > task->finalTime) {
      result = true;
    } else {
      result = isEvent(task->eventCode);
    }
  } else {
    if (t > task->finalTime) {
      result = true;
    } else {
      // printf("DEBUG: t=%d\n", task->finalTime-t);
    }
  }
  return result;
}

bool CER1Robot::moveLine(CTask *task) {
  bool result = false;
  if (!task->initialized) {
    if (!rcmPower) {
      enableMotion();
    }
    readOdometry();
    task->tresholdLine[0] = cos(h);
    task->tresholdLine[1] = sin(h);
    if (task->length < 0) {
      task->tresholdLine[0] = -task->tresholdLine[0];
      task->tresholdLine[1] = -task->tresholdLine[1];
    }
    task->tresholdLine[2] =
        -(task->tresholdLine[0] * (x + task->length * cos(h)) +
          task->tresholdLine[1] * (y + task->length * sin(h)));
    printf("DEBUG: LINE len=%.3f a=%.3f b=%.3f c=%.3f\n", task->length,
           task->tresholdLine[0], task->tresholdLine[1], task->tresholdLine[2]);
    task->speed = sign(task->length) * fabs(task->speed);
    rcm.setVelocities(task->speed, -task->speed, true);
    task->initialized = true;
  } else {
    readOdometry();
    double correction = 0.84 * fabs(task->speed);
    double t = (task->tresholdLine[0] * x + task->tresholdLine[1] * y +
                task->tresholdLine[2]);
    printf("DEBUG: t=%.3f\n", t);
    if (t > 0 - correction) {
      rcm.setVelocities(0.0, 0.0, true);
      result = true;
    }
  }
  return result;
}

bool CER1Robot::moveCurve(CTask *task) {
  bool result = true;
  printf("DEBUG: CURVE not implemented !!!\n");
  return result;
}

bool CER1Robot::moveTurnTo(CTask *task) {
  bool result = false;
  if (!task->initialized) {
    if (!rcmPower) {
      enableMotion();
    }
    readOdometry();
    task->finalAngle = task->angle;
    double speed = fabs(task->steeringSpeed);
    double dh = task->finalAngle - h;
    normalizeAngle(&dh);
    printf("DEBUG: TURN TO %.3f   dh=%.3f\n", task->finalAngle, dh);
    if (dh < 0) {
      speed = -speed;
    }
    task->steeringSpeed = speed;
    rcm.setVelocities(speed, speed, true);
    task->initialized = true;
  } else {
    readOdometry();
    double dh = task->finalAngle - h;
    normalizeAngle(&dh);
    printf("DEBUG: dh=%.3f\n", dh);
    if (task->steeringSpeed > 0) {
      // left
      if (dh <= 0.0) {
        result = true;
      }
    } else {
      // right
      if (dh >= 0.0) {
        result = true;
      }
    }
  }
  return result;
}

bool CER1Robot::moveTurnBy(CTask *task) {
  bool result = false;
  if (!task->initialized) {
    if (!rcmPower) {
      enableMotion();
    }
    readOdometry();
    task->finalAngle = h + task->angle;
    double speed = fabs(task->steeringSpeed);
    double dh = task->finalAngle - h;
    normalizeAngle(&dh);
    printf("DEBUG: TURN BY %.3f   dh=%.3f\n", task->finalAngle, dh);
    if (dh < 0) {
      speed = -speed;
    }
    task->steeringSpeed = speed;
    rcm.setVelocities(speed, speed, true);
    task->initialized = true;
  } else {
    readOdometry();
    double correction = fabs(task->steeringSpeed) * 0.84;
    double dh = task->finalAngle - h;
    normalizeAngle(&dh);
    printf("DEBUG: t=%d   dh=%.3f\n", getTime(), dh);
    if (task->steeringSpeed > 0) {
      // left
      if (dh <= 0.0 + correction) {
        result = true;
      }
    } else {
      // right
      if (dh >= 0.0 - correction) {
        result = true;
      }
    }
    if (result) {
      rcm.setVelocities(0.0, 0.0, true);
      readOdometry();
      double dh = task->finalAngle - h;
      normalizeAngle(&dh);
      printf("DEBUG: final error = %.3f\n", dh);
    }
  }
  return result;
}

void CER1Robot::processVelocityControlMode() {
  if (fabs(leftWheelVelocity) > 0.001 || fabs(rightWheelVelocity) > 0.001) {
    lastVelocityCommandTime = getTime();
    if (!rcmPower) {
      // fprintf(stderr, "enable motion\n"); fflush(stderr);
      enableMotion();
    }
    readOdometry();
    long t0 = getTime();
    rcm.setVelocities(rightWheelVelocity, -leftWheelVelocity, true);
    fprintf(stderr, "ER1 velocity move dt=%d ms\n", getTime() - t0);
    fflush(stderr);
  } else {
    // fprintf(stderr, "velocity control mode no velocity power=%d\n",
    // rcmPower); fflush(stderr);
    if (rcmPower) {
      rcm.setVelocities(0.0, 0.0, true);
    }
    long dt = getTime() - lastVelocityCommandTime;
    if (dt > 1000) {
      disableMotion();
    }
  }
}
