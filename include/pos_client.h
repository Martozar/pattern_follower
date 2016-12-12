/*
 * File name: pos_client.h
 * Date:      2012/08/07 15:23
 * Author:    Jan Chudoba
 */

#ifndef __POS_CLIENT_H__
#define __POS_CLIENT_H__

#include "text_socket.h"

#define DATA_HEADER "DATA"
#define CONTROL_MESSAGE_HEADER "CONTROL"
#define PING_MESSAGE_HEADER "PING"
#define ECHO_MESSAGE_HEADER "ECHO"

class CPositionMessage
{
   public:
	uint8_t flags;
	int32_t x; // [mm]
	int32_t y;
	int32_t z;
	int32_t a; // angle [1000 * rad]
	int32_t b;
	int32_t c;
	uint32_t timestamp;
};

class CControlMessage
{
	public:
	int32_t forwardVelocity;
	int32_t angularVelocity;
};

class CPingMessage
{
	public:
	uint32_t pingTimer;
	uint32_t echoTimer;
};

void client_receive_data(CTextSocket*);

class CPositionClient
{
   CClientTextSocket s;
   void (*data_callback) (CPositionMessage*);
   bool connected_to_server;
	uint32_t timeDifference;
	bool timeDifferenceInitialized;

public:
   CPositionClient(const char * serverIp, int port, void (*callback)(CPositionMessage*));
   ~CPositionClient();

   bool connected() { return connected_to_server; }

	bool sendPing();
	bool sendControl(int forward, int angular);

protected:
   friend void client_receive_data(CTextSocket*);
   void receiveData();

	void info(const char * fmt, ...);
	void debug(const char * fmt, ...);
};

#endif

/* end of pos_client.h */
