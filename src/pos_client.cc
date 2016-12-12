/*
 * File name: pos_client.cc
 * Date:      2012/08/07 15:26
 * Author:    Jan Chudoba
 */

#include <stdio.h>
#include <stdarg.h>
#include <sys/time.h>

#include <pattern_follower/pos_client.h>

#define error info

static uint32_t getTimeMs()
{
	uint32_t t;
	struct timeval now;
	gettimeofday(&now, NULL);
	t = now.tv_sec * 1000 + now.tv_usec / 1000;
	return t;
}

CPositionClient::CPositionClient(const char * serverIp, int port, void (*callback)(CPositionMessage*))
{

    printf(serverIp);
   data_callback = callback;
	timeDifference = 0;
	timeDifferenceInitialized = false;
   s.setContext(this);
   s.setReceiveCallback(client_receive_data);
   uint32_t serverAddress = convertIpAddress((char*)serverIp);

   connected_to_server = s.open((uint8_t*)&serverAddress, port);

   if (connected_to_server) {
      info("connected to server");
   } else {
      error("failed to connect to server");
   }
}

CPositionClient::~CPositionClient()
{
   s.close();
}

bool CPositionClient::sendPing()
{
	CPingMessage ping;
	ping.pingTimer = getTimeMs();
	ping.echoTimer = 0;
	char msg[100];
	memcpy(msg, PING_MESSAGE_HEADER, sizeof(PING_MESSAGE_HEADER));
	int length = sizeof(PING_MESSAGE_HEADER);
	memcpy(msg+length, &ping, sizeof(CPingMessage));
	length += sizeof(CPingMessage);
	return s.send((uint8_t*)msg, length);
}

bool CPositionClient::sendControl(int forward, int angular)
{
	CControlMessage ctrl;
	ctrl.forwardVelocity = forward;
	ctrl.angularVelocity = angular;
	char msg[100];
	memcpy(msg, CONTROL_MESSAGE_HEADER, sizeof(CONTROL_MESSAGE_HEADER));
	int length = sizeof(CONTROL_MESSAGE_HEADER);
    memcpy(msg+length, &ctrl, sizeof(CControlMessage));
	length += sizeof(CControlMessage);
//	msg = {'\0'};
	return s.send((uint8_t*)msg, length);
}

void client_receive_data(CTextSocket * s)
{
   CPositionClient * client = (CPositionClient*) s->getContext();
   if (client) {
      client->receiveData();
   }
}

void CPositionClient::receiveData()
{
   if (s.available() > 0) {
      char buffer[128];
      int r = s.receive((uint8_t*)buffer, 128);
      if (r > 0) {
         buffer[r] = 0;
         if (strcmp((const char*)buffer, DATA_HEADER) == 0) {
            debug("received data, size=%d", r);
            // for (int i=0; i<r; i++) { printf(" %d (%c)", (int)buffer[i], buffer[i]); } printf("\n");
            CPositionMessage * pos = (CPositionMessage*)(buffer + sizeof(DATA_HEADER));
            if (data_callback) {
					if (timeDifferenceInitialized) {
						pos->timestamp += timeDifference;
					} else {
						pos->timestamp = 0;
					}
               data_callback(pos);
            }
         } else if (strcmp((const char*)buffer, "HELLO") == 0) {
            info("server responded to connection");
         } else
         if (strcmp((const char*)buffer, ECHO_MESSAGE_HEADER) == 0) {
				uint32_t t = getTimeMs();
				CPingMessage * echo = (CPingMessage*)(buffer + sizeof(ECHO_MESSAGE_HEADER));
				uint32_t dt = t - echo->pingTimer;
				timeDifference = t - echo->echoTimer - dt/2;
				timeDifferenceInitialized = true;
            info("echo dt=%u ms", dt);
         } else {
            info("unrecognized message '%s'", buffer);
         }
      }
   }
}

#define DEBUG_STR  "pos_client DEBUG: "
#define INFO_STR   "pos_client INFO:  "

void CPositionClient::info(const char * fmt, ...)
{
	va_list arg;
	va_start(arg, fmt);
	{
      //printTimestamp();
      fprintf(stderr, INFO_STR);
      vfprintf(stderr, fmt, arg);
      fprintf(stderr, "\n");
	}
	va_end(arg);
}

void CPositionClient::debug(const char * fmt, ...)
{
	va_list arg;
	va_start(arg, fmt);
	{
      //printTimestamp();
      fprintf(stderr, DEBUG_STR);
      vfprintf(stderr, fmt, arg);
      fprintf(stderr, "\n");
	}
	va_end(arg);
}


/* end of pos_client.cc */
