/*
 * File name: text_socket.cc
 * Date:      2010/05/05 15:50
 * Author:    Jan Chudoba
 *
 * Based on:  rmi_socket (from obc/robot_monitor_interface)
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#include <pattern_follower/text_socket.h>

#define INET_ADDRESS_LENGTH 4

#ifdef TEXT_SOCKET_DEBUG
	#define text_socket_debug(...) text_socket_debug_print(__VA_ARGS__)
#else
	#define text_socket_debug(...)
#endif

#define text_socket_info(...) text_socket_debug_print(__VA_ARGS__)

void text_socket_debug_print(const char* text, ...)
{
	va_list arg;
	va_start(arg, text);
	fprintf(stderr, "TEXT_SOCKET DEBUG: ");
	vfprintf(stderr, text, arg);
	fprintf(stderr, "\n");
	va_end(arg);
}

char* print_ip_address(char* result, uint8_t* addr);

const char* socket_error_str[] = {
	"OK",
	"SOCKET_ERROR_CREATE",
	"SOCKET_ERROR_BIND",
	"SOCKET_ERROR_LISTEN",
	"SOCKET_ERROR_ACCEPT",
	"SOCKET_ERROR_CONNECT"
};

typedef enum {
	SLIP_END = 0xC0,
	SLIP_ESC = 0xDB,
	SLIP_ESC_END = 0xDC,
	SLIP_ESC_ESC = 0xDD
} TSlipCharacterss;

void socketDisableSigPipe()
{
	struct sigaction sa;
	sa.sa_handler = SIG_IGN;
	sa.sa_flags = 0;
	if (sigemptyset(&sa.sa_mask) == -1 || sigaction(SIGPIPE, &sa, 0) == -1) {
		perror("failed to ignore SIGPIPE; sigaction");
		exit(EXIT_FAILURE);
	} else {
		text_socket_debug("disabled SIGPIPE signal");
	}
}

static sigset_t signal_mask;

void* text_socket_thread(void* param) { return ((CTextSocket*)param)->threadBody(); }

CTextSocket::CTextSocket() :
   thread(text_socket_thread, this)
{
	sockfd = -1;
	last_error = SOCKET_ERROR_OK;
	receive_length = 0;
	pthread_mutex_init(&receive_buffer_mutex, NULL);
	receive_callback_function = NULL;
	accept_callback_function = NULL;
	disconnect_callback_function = NULL;
	context = NULL;

	sigemptyset(&signal_mask);
	sigaddset(&signal_mask, SIGPIPE);
}

CTextSocket::~CTextSocket()
{
	close();
	pthread_mutex_destroy(&receive_buffer_mutex);
}

const char* CTextSocket::getLastErrorStr()
{
	return socket_error_str[last_error];
}

bool CTextSocket::open()
{
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		last_error = SOCKET_ERROR_CREATE;
		return false;
	}
#if 0 // not portable
	int set = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_NOSIGPIPE, (void *)&set, sizeof(int)); // disable SIGPIPE signal on writing into broken pipe
#endif
	text_socket_debug("CTextSocket opened socket, fd=%d", sockfd);
	return true;
}

bool CTextSocket::close()
{
	stop();
	if (sockfd>=0) {
		text_socket_debug("CTextSocket close socket (fd=%d)", sockfd);
		::close(sockfd);
		sockfd = -1;
	}
	return true;
}

bool CTextSocket::isConnected()
{
	return (sockfd >= 0);
}

bool CTextSocket::send(uint8_t *data, int length)
{
	uint8_t buffer[2*TEXT_SOCKET_MAX_MESSAGE_SIZE+2];
	int len = 0;
	buffer[len++] = SLIP_END;
	for (int i=0; i<length; i++) {
		switch(data[i]) {
		case SLIP_END:
			buffer[len++] = SLIP_ESC;
			buffer[len++] = SLIP_ESC_END;
			break;
		case SLIP_ESC:
			buffer[len++] = SLIP_ESC;
			buffer[len++] = SLIP_ESC_ESC;
			break;
		default:
			buffer[len++] = data[i];
			break;
		}
	}
	buffer[len++] = SLIP_END;
	int wb;
	try {
		//wb = write(sockfd, buffer, len);
		wb = ::send(sockfd, buffer, len, 0);
		if (wb == -1) {
			text_socket_debug("ERROR: write on CTextSocket(%d) returned errno %s", sockfd, strerror(errno));
			// handle EPIPE
			if (errno == EPIPE) {
				text_socket_debug("ERROR: CTextSocket(%d) BROKEN PIPE", sockfd);
            ::shutdown(sockfd, SHUT_RD);
				sockfd = -1;
			}
		} else {
			text_socket_debug("CTextSocket (%d) send message length=%d, sent=%d", sockfd, len, wb);
		}
	} catch (int e) {
		wb = -1;
		text_socket_debug("EXCEPTION %d catched on CTextSocket(%d)/write(), errno=%s", e, sockfd, strerror(errno));
	}
	return (wb == len);
}

int CTextSocket::receive(uint8_t *data, int length)
{
	if (length > receive_length) {
		length = receive_length;
	}
	pthread_mutex_lock(&receive_buffer_mutex);
	memcpy(data, receive_buffer, length);
	pthread_mutex_unlock(&receive_buffer_mutex);
	text_socket_debug("CTextSocket received %d bytes:", length);
#ifdef TEXT_SOCKET_DEBUG
	for (int i=0; i<length; i++) printf(" %02X", data[i]);
	printf("\n");
#endif
	return length;
}

int CTextSocket::available()
{
	return receive_length;
}

void CTextSocket::stop()
{
	text_socket_debug("CTextSocket (%d) stop()", sockfd);
   thread.interrupt();
	if (sockfd >= 0) {
		text_socket_debug("socket (%d) shutdown()", sockfd);
		::shutdown(sockfd, SHUT_RDWR);
	}
	thread.join();
}

void CTextSocket::shutdown()
{
	stop();
}

void* CTextSocket::threadBody()
{
	//socketDisableSigPipe();
	text_socket_info("CTextSocket (%d) thread started", sockfd);
	int sigmask_ret = pthread_sigmask(SIG_BLOCK, &signal_mask, NULL);
	if (sigmask_ret != 0) {
		text_socket_info("CTextSocket (%d) pthread_sigmask() returned with error %d", sockfd, sigmask_ret);
	}
   bool q = thread.shouldInterrupt();
	uint8_t last = 0;
	bool supress_buffer_warning = false;
   while (!q) {
		uint8_t buf[1];
		//text_socket_debug("CTextSocket (%d) read ...", sockfd);
		int rb = read(sockfd, buf, 1);
		if (rb == 1) {
			//text_socket_debug("CTextSocket (%d) received byte %d (%02X)", sockfd, buf[0], buf[0]);
			uint8_t c = buf[0];
			bool data_valid = false;
			uint8_t data_byte;
			if (last != SLIP_ESC) {
				if (c == SLIP_END) {
					if (receive_length > 0) {
						text_socket_debug("CTextSocket received message (len = %d bytes, cb=%c)", receive_length, (receive_callback_function)?'Y':'N');
						if (receive_callback_function) {
							receive_callback_function(this);
						}
					}// else text_socket_debug("X");
					receive_length = 0;
					supress_buffer_warning = false;
				} else
				if (c != SLIP_ESC) {
					data_byte = c;
					data_valid = true;
				}
			} else {
				switch (c) {
				case SLIP_ESC_END:
					data_byte = SLIP_END;
					data_valid = true;
					break;
				case SLIP_ESC_ESC:
					data_byte = SLIP_ESC;
					data_valid = true;
					break;
				default:
					text_socket_debug("ERROR: CTextSocket slip protocol error");
				}
			}
			if (data_valid) {
				if (receive_length < TEXT_SOCKET_MAX_MESSAGE_SIZE) {
					pthread_mutex_lock(&receive_buffer_mutex);
					receive_buffer[receive_length++] = data_byte;
					pthread_mutex_unlock(&receive_buffer_mutex);
				} else {
					if (!supress_buffer_warning) {
						text_socket_debug("ERROR: CTextSocket (%d) message length exceeds buffer size", sockfd);
					}
				}
			}
			last = c;
		} else if (rb == 0) {
			// EOF
			text_socket_info("CTextSocket (%d) received 0 bytes - EOF", sockfd);
			q = true;
		} else {
			// READ ERROR
			text_socket_info("CTextSocket (%d) receive - read error %d (errno=%s)", sockfd, rb, strerror(errno));
         q = true;
		}
      {
         //ScopedLock lk(mtx);
         q = q || thread.shouldInterrupt();
      }
		//text_socket_debug("q=%d, interrupt_flag=%d", q, interrupt_flag);
   }
	text_socket_info("CTextSocket (%d) thread stopped", sockfd);
	if (disconnect_callback_function) disconnect_callback_function(this);
   return 0;
}

//void CTextSocket::startThread()
//{
	//thread.start();
//}

// ================================================================================
// CServerTextSocket
// ================================================================================

bool CServerTextSocket::open(uint16_t server_port)
{
	port = server_port;
	if (!CTextSocket::open()) return false;

	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = INADDR_ANY;
	server_addr.sin_port = htons(port);

	if (bind(sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
      text_socket_debug("CServerTextSocket bind error, fd=%d", sockfd);
		close();
		last_error = SOCKET_ERROR_BIND;
		return false;
	}

	listen(sockfd, 5);

	thread.start();

	return true;
}

bool CServerTextSocket::waitForClient()
{
	struct sockaddr_in cli_addr;

	int clilen = sizeof(cli_addr);
	int newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t*)&clilen);
	if (newsockfd < 0) {
		text_socket_debug("CServerTextSocket::waitForClient() error (accept, errorcode=%d/%s)", errno, strerror(errno));
		last_error = SOCKET_ERROR_ACCEPT;
		return false;
	}
	text_socket_debug("CServerTextSocket client accepted, new socket fd=%d", newsockfd);

	CClientTextSocket *client = new CClientTextSocket();
	client->setContext(context);
	client->setReceiveCallback(receive_callback_function);
	client->setDisconnectCallback(disconnect_callback_function);
	client->set(newsockfd, &cli_addr);

	if (accept_callback_function) {
		accept_callback_function(client);
	}

	char tmp[20] __attribute__ (( unused ));
	text_socket_debug("CServerTextSocket client accepted, address=%s", print_ip_address(tmp, (uint8_t*)&cli_addr.sin_addr.s_addr ));

	return true;
}

void* CServerTextSocket::threadBody()
{
	text_socket_debug("CServerTextSocket thread started");
   bool q = thread.shouldInterrupt();
   while (!q) {
		waitForClient();
      {
         //ScopedLock lk(mtx);
         q = thread.shouldInterrupt();
      }
   }
	text_socket_debug("CServerTextSocket thread stopped");
   return 0;
}

// ================================================================================
// CClientTextSocket
// ================================================================================

bool CClientTextSocket::open(uint8_t *server_address, uint16_t server_port)
{

	if (!CTextSocket::open()) return false;

	if (server_address) {
		bzero((char *) &server_addr, sizeof(server_addr));
		server_addr.sin_family = AF_INET;
		bcopy((char *)server_address, (char *)&server_addr.sin_addr.s_addr, INET_ADDRESS_LENGTH);
		server_addr.sin_port = htons(server_port);
	}

	text_socket_debug("CClientTextSocket::open() connect() ...\n");

	if (connect(sockfd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
#ifdef TEXT_SOCKET_DEBUG
		uint8_t* a = (uint8_t*) &server_addr.sin_addr.s_addr;
		text_socket_debug("socket connect(fd=%d, addr=%d.%d.%d.%d:%d) failed.", sockfd, a[0], a[1], a[2], a[3], ntohs(server_addr.sin_port));
#endif
		close();
		last_error = SOCKET_ERROR_CONNECT;
		return false;
	}

	//text_socket_debug("CClientTextSocket::open() execute() ...");

   thread.start();
	//text_socket_debug("CClientTextSocket::open() execute() done.");

	return true;
}

bool CClientTextSocket::reopen()
{
	return open(NULL, 0);
}

void CClientTextSocket::set(int fd, struct sockaddr_in * addr)
{
	text_socket_debug("created server client connection, fd=%d", fd);
	sockfd = fd;
	bcopy((char*) addr, (char*)&server_addr, sizeof(server_addr));
   thread.start();
}

// ================================================================================

char* print_ip_address(char* result, uint8_t* addr)
{
	sprintf(result, "%u.%u.%u.%u", addr[0], addr[1], addr[2], addr[3]);
	return result;
}

uint32_t convertIpAddress(char* strIpAddress)
{
	uint32_t addr = 0;
	uint8_t * pAddr = (uint8_t*) &addr;
	for (int i=0; i<4; i++) {
		uint8_t a = 0;
		while (*strIpAddress != 0) {
			char c = *strIpAddress++;
			if (c=='.') break;
			uint8_t n = c-'0';
			if (n>=10) return 0;
			a = 10*a + n;
		}
		pAddr[i] = a;
	}
	return addr;
}

bool convertIpAddress(uint8_t* ipaddress, char* strIpAddress)
{
	uint32_t ip = convertIpAddress(strIpAddress);
	memcpy(ipaddress, &ip, 4);
	return (ip != 0);
}

/* end of text_socket.cc */
