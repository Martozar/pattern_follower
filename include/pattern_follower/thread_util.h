/*
 * File name: thread_util.h
 * Date:      2011/01/07 10:15
 * Author:    Jan Chudoba
 */

#ifndef __THREAD_UTIL_H__
#define __THREAD_UTIL_H__

#include <pthread.h>
#include <signal.h>

class CThreadObject {
private:
  pthread_t thread_id;
  bool interruptFlag;
  void *(*thread_body)(void *);
  void *arg;

public:
  CThreadObject(void *(*thread_body)(void *), void *arg);
  ~CThreadObject();

  /* start new thread */
  bool start();

  /* called from thread body to check interrupt request */
  bool shouldInterrupt() { return interruptFlag; }

  /* interrupt request */
  void interrupt(bool kill_signal = false) {
    interruptFlag = true;
    if (kill_signal)
      kill(SIGINT);
  }

  void join();

  /* interrupt() and join() */
  void stop();

  bool isRunning() { return (thread_id != 0); }

  // int getId() { return thread_id; }

  int kill(int sig);
};

class CMutex {
  pthread_mutex_t mutex;

public:
  CMutex();
  ~CMutex();

  void lock();
  void unlock();
  bool trylock();
};

class CThreadEvent {
private:
  pthread_cond_t cond;
  pthread_mutex_t mutex;
  bool eventState;

public:
  CThreadEvent();
  ~CThreadEvent();

  bool wait();
  bool wait(int timeout); // timeout is in milliseconds
  void notify();
  void reset();

  bool getState() { return eventState; }
};

#endif

/* USAGE:
 *
 * CThreadObject thread(thread_body_fcn, context);
 *
 * thread.start();
 * ...
 * thread.stop();
 *
 * void * thread_body_fcn(void * context)
 * {
 *    while (! thread.shouldInterrupt()) {
 *       ... do something
 *    }
 *    return 0;
 * }
 *
 *
 */

/* end of thread_util.h */
