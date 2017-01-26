/*
 * File name: thread_util.cc
 * Date:      2011/01/07 10:21
 * Author:    Jan Chudoba
 */

#include <errno.h>
#include <pattern_follower/thread_util.h>
#include <stdio.h>
#include <string.h>

// =============================================================================
// CThreadObject
// =============================================================================

CThreadObject::CThreadObject(void *(*thread_body)(void *), void *arg)
    : thread_body(thread_body), arg(arg) {
  interruptFlag = false;
  thread_id = 0;
}

CThreadObject::~CThreadObject() { stop(); }

bool CThreadObject::start() {
  if (thread_id == 0) {
    interruptFlag = false;
    return (pthread_create(&thread_id, 0, thread_body, arg) == 0);
  } else {
    return false;
  }
}

void CThreadObject::join() {
  if (thread_id != 0) {
    pthread_join(thread_id, NULL);
    thread_id = 0;
  }
}

void CThreadObject::stop() {
  // printf("CThreadObject::stop() thread_id=%d\n", thread_id);
  if (thread_id != 0) {
    interrupt(true);
    join();
  }
  // printf("CThreadObject::stop() done.\n");
}

int CThreadObject::kill(int sig) {
  if (thread_id != 0) {
    return pthread_kill(thread_id, sig);
  } else {
    return -1;
  }
}

// =============================================================================
// CMutex
// =============================================================================

CMutex::CMutex() { pthread_mutex_init(&mutex, NULL); }

CMutex::~CMutex() { pthread_mutex_destroy(&mutex); }

void CMutex::lock() { pthread_mutex_lock(&mutex); }

void CMutex::unlock() { pthread_mutex_unlock(&mutex); }

bool CMutex::trylock() { return (pthread_mutex_trylock(&mutex) == 0); }

// =============================================================================
// CThreadEvent
// =============================================================================

CThreadEvent::CThreadEvent() {
  eventState = false;
  pthread_mutex_init(&mutex, NULL);
  pthread_cond_init(&cond, NULL);
}

CThreadEvent::~CThreadEvent() {
  pthread_cond_destroy(&cond);
  pthread_mutex_destroy(&mutex);
}

bool CThreadEvent::wait() {
  int rc = 0;
  pthread_mutex_lock(&mutex);
  while (!eventState) {
    rc = pthread_cond_wait(&cond, &mutex);
  }
  pthread_mutex_unlock(&mutex);
  return (rc == 0);
}

bool CThreadEvent::wait(int timeout) {
  int rc = 0;
  // printf("%p: wait lock\n", this);
  pthread_mutex_lock(&mutex);
  // printf("%p: wait locked\n", this);
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += timeout / 1000;
  ts.tv_nsec += 1000000 * (timeout % 1000);
  if (ts.tv_nsec > 1000000000) {
    ts.tv_sec += ts.tv_nsec / 1000000000;
    ts.tv_nsec = ts.tv_nsec % 1000000000;
  }
  while (!eventState) {
    // printf("%p: wait cond_timedwait\n", this);
    rc = pthread_cond_timedwait(&cond, &mutex, &ts);
    if (rc != 0) {
      if (rc != ETIMEDOUT)
        printf("thread_util: wait(int) pthread_cond_timedwait returned error "
               "%d, errno=%s\n",
               rc, strerror(errno));
      break;
    }
    // printf("%p: wait cond_timedwait done\n", this);
  }
  // printf("%p: wait unlock\n", this);
  pthread_mutex_unlock(&mutex);
  return (rc == 0);
}

void CThreadEvent::notify() {
  pthread_mutex_lock(&mutex);

  eventState = true;
  pthread_cond_broadcast(&cond);

  pthread_mutex_unlock(&mutex);
}

void CThreadEvent::reset() {
  pthread_mutex_lock(&mutex);
  eventState = false;
  pthread_mutex_unlock(&mutex);
}

/* end of thread_util.cc */
