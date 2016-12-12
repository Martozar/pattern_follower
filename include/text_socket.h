/*
 * File name: text_socket.h
 * Date:      2011/03/27
 * Author:    Jan Chudoba
 *
 * Based on:  rmi_socket
 */

#ifndef __TEXT_SOCKET_H__
#define __TEXT_SOCKET_H__

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <pthread.h> // used for mutex
#include "thread_util.h"

typedef enum {
	SOCKET_ERROR_OK,
	SOCKET_ERROR_CREATE,
	SOCKET_ERROR_BIND,
	SOCKET_ERROR_LISTEN,
	SOCKET_ERROR_ACCEPT,
	SOCKET_ERROR_CONNECT,
	SOCKET_ERROR_NUMBER // last
} TTextSocketError;

void socketDisableSigPipe();

#define TEXT_SOCKET_MAX_MESSAGE_SIZE 1024

class CTextSocket
{
protected:
	int port;
	int sockfd;
	struct sockaddr_in server_addr;

	TTextSocketError last_error;

   CThreadObject thread;

	uint8_t receive_buffer[TEXT_SOCKET_MAX_MESSAGE_SIZE];
	int receive_length;
	pthread_mutex_t receive_buffer_mutex;
	void (*receive_callback_function)(CTextSocket*);
	void (*accept_callback_function)(CTextSocket*);
	void (*disconnect_callback_function)(CTextSocket*);

	void* context;
public:
	CTextSocket();
	virtual ~CTextSocket();

	TTextSocketError getLastError() { return last_error; }
	const char* getLastErrorStr();

	virtual bool open();
	virtual bool close();
	virtual bool isConnected();

	bool send(uint8_t *data, int length);
	int receive(uint8_t *data, int length);
	bool send(const char* str) { return send((uint8_t*)str, strlen(str)+1);}
	int receive(char *str, int max_length) { return receive((uint8_t*)str, max_length); }
	int available(); // WARNING: check available only in receive callback function
	void setReceiveCallback(void (*fce)(CTextSocket*)) { receive_callback_function = fce; }
	void setAcceptCallback(void (*fce)(CTextSocket*)) { accept_callback_function = fce; }
	void setDisconnectCallback(void (*fce)(CTextSocket*)) { disconnect_callback_function = fce; }

	void setContext(void* ptr) { context = ptr; }
	void* getContext() { return context; }
	int getSockFd() { return sockfd; }
	int getPort() { return ntohs(server_addr.sin_port); }
	void getAddress(uint8_t* address) { memcpy(address, &server_addr.sin_addr.s_addr, 4); }

	// thread methods
	void stop(void);
	virtual void shutdown(void);
	virtual void* threadBody(void);
protected:
	//void startThread();
};

class CServerTextSocket : public CTextSocket
{
public:
	virtual bool open(uint16_t server_port);

	bool waitForClient();
	virtual void* threadBody(void);
};

class CClientTextSocket : public CTextSocket
{
public:
	virtual bool open(uint8_t *server_address, uint16_t server_port);
	virtual bool reopen();

	void set(int fd, struct sockaddr_in * addr);
};

char* print_ip_address(char* result, uint8_t* addr);
uint32_t convertIpAddress(char* strIpAddress);
bool convertIpAddress(uint8_t* ipaddress, char* strIpAddress);

#endif

/* end of text_socket.h */
