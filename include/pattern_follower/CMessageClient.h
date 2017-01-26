#ifndef CMESSAGECLIENT_H
#define CMESSAGECLIENT_H

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pattern_follower/CMessage.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

/**
@author Tom Krajnik
*/
class CMessageClient {
public:
  CMessageClient();
  ~CMessageClient();

  int init(const char *ip, const int port, bool requirements[]);
  int checkForHeader();
  int checkForData(double odo[], bool but[], int rotat[]);
  int checkForStatus(
      CStatusMessage &status); // don't forget to call checkForHeader() before
  int sendMessage(CMessage &message);

private:
  int checkForInts(int data[], unsigned int len);
  int checkForBools(bool data[], unsigned int len);
  int checkForDoubles(double data[], unsigned int len);

  //  TLogModule module;
  int mySocket;
};

#endif
