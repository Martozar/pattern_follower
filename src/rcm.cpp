/*
 * File name: rcm.cpp
 * Date:      2005/02/10 15:39
 * Author:    Jan Fail
 */

#include <fcntl.h>

#include <pattern_follower/rcm.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <termios.h>

const HANDLE RCM::INVALID_HANDLE=-1;

#define HIBYTE(x) ((x>>8) & 0xff)
#define LOBYTE(x) (x&0xff)
#define HIWORD(x) ((x>>16) & 0xffff)
#define LOWORD(x) (x&0xffff)

//??
#define MAKEWORD(x,y) ((y<<8)|x)
#define MAKELONG(x,y) ((y<<16)|x)

#define RETRY_NUMBER 10
//in us
#define RETRY_TIMEOUT 10000

static bool exit_flag = false;
/*----------------------------------------------------------------------------*/
long GetTickCount() {
   unsigned int ret;
   struct timeval t_time;
   gettimeofday(&t_time, NULL);
//   fprintf(stdout, "GetTickCount sec:%d usec %d", t_time.tv_sec, t_time.tv_usec);
   ret = t_time.tv_sec*1000 + t_time.tv_usec/1000;
   return ret;
}

/*----------------------------------------------------------------------------*/
//sleep in ms
void Sleep(int miliseconds) {
   usleep(miliseconds*1000);
}

/*----------------------------------------------------------------------------*/
//Debug Log
/*----------------------------------------------------------------------------*/
char * tmpLine = NULL;

/*----------------------------------------------------------------------------*/
bool ReadFile(HANDLE hSerial, void * i_buffer, int number_to_read, DWORD * number_readed, DWORD * overlapped) {
	bool ret = false;
	BYTE * buffer = (byte*)i_buffer;
	int readed = 0;
	int retry = 0;
	int available;
	int readcount = 0;
	int toread  = number_to_read;
	do {
		if (exit_flag) {
			//return false;
			return true;
		}
		ioctl(hSerial, FIONREAD, &available);
		if (available < 1) {
			usleep(RETRY_TIMEOUT);
			retry++;
		}
		ioctl(hSerial, FIONREAD, &available);
		if (available>0) {
			toread = number_to_read - readed;
			if (available < toread) {
				toread = available;
			}
			readcount = read(hSerial, &buffer[readed], toread);
			if (readcount <= 0) {
				return false;
			} else {
				readed += readcount;
				toread -= readcount;
			}
			if (readed >= number_to_read) {
				*number_readed = readed;
				return true;
			}
		}
	} while (retry < RETRY_NUMBER);
	return false;
}

/*----------------------------------------------------------------------------*/
//write to hSerial
bool WriteFile(HANDLE hSerial, const void * i_buffer, int number_to_write, DWORD * number_written, DWORD * overlapped) {
   bool ret = false;
   int retry = 0;
   int status;

   /*   ioctl(hSerial, TIOCMGET, &status);
	while(!(status & TIOCM_CTS))  {
	if (exit_flag) {
	printDump( "EXIT FLAG\n");
	return false;
	}
	printDump( "wait\n");
	usleep(RETRY_TIMEOUT);
	ioctl(hSerial, TIOCMGET, &status);
	if (retry >= RETRY_NUMBER) {
	printDump( "RCM is not ready\n");
	return false;
	}
	} */
   byte * p = (byte*)i_buffer;
   (*number_written) = write(hSerial, i_buffer, number_to_write);
   tcdrain(hSerial);
   return true;
}


/*----------------------------------------------------------------------------
 * Class RCM
 *----------------------------------------------------------------------------*/
// PIMP implementation
int PIMP_COMMAND::UpdateChecksum(int cbData)
{
   assert(cbData >= 0 && cbData <= 6);
   byte sum = address + axis + opcode;
   for (int i = 0; i < cbData; i++)
      sum += data[i];
   checksum = -sum;
   return PIMP_COMMAND_SIZE(cbData);
}

bool PIMP_RESPONSE::CheckChecksum(int cbData)
{
   assert(cbData >= 0 && cbData <= 6);
   byte sum = status + checksum;
   for (int i = 0; i < cbData; i++)
      sum += data[i];
   return sum == 0;
}


int closeHandle(HANDLE m_hSerial) {
   return close(m_hSerial);
}

///////////////////////////////////////////////////////////////
// serial helpers

static int OpenSerial(char * iPort)
{
   char szPort[32];
   // direct device control
   int h =  open(iPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
   if (h == -1) {
      printf( "Can not open port %s", iPort);
   }
   return h;
}


#define STTY_PATTERN "stty -F %s 230400 -parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff  -iuclc -ixany -imaxbel -opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0   ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop  -echoprt -echoctl -echoke"

//static bool setCommDefaults(HANDLE hSerial)
//device is path to serial port
static bool setCommDefaults(char * device)
{
   //  if (hSerial == RCM::INVALID_HANDLE) return false;
   char cmd[MAX_CHARS];
   bool ret = true;
   sprintf(cmd, STTY_PATTERN, device);
   if (system(cmd)) {
      ret = false;
   }


   /*  DCB        dcb;
       memset(&dcb, 0, sizeof(DCB));
       dcb.DCBlength = sizeof(DCB);

   // Guesses
   dcb.fBinary = TRUE;
   dcb.fParity = FALSE;
   dcb.BaudRate = 250000;      // default
   dcb.ByteSize = 8;
   dcb.Parity = NOPARITY;
   dcb.StopBits = ONESTOPBIT;

   dcb.fOutxDsrFlow = FALSE;
   dcb.fDtrControl = DTR_CONTROL_DISABLE;
   dcb.fOutxCtsFlow = FALSE;
   dcb.fRtsControl = RTS_CONTROL_DISABLE;
   dcb.fInX = dcb.fOutX = FALSE;

   if (!setCommState(hSerial, &dcb))
   return false;

#if 1
   // set no timeouts
   COMMTIMEOUTS to;
   memset(&to, 0, sizeof(to));
   to.ReadIntervalTimeout = MAXDWORD;  // return immediately if no data
   if (!setCommTimeouts(hSerial, &to))
   return false;
#endif

   // purge all
   PurgeComm(hSerial,
   PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
   */
   return ret;
}

static bool PeekSerialByte(HANDLE hSerial, BYTE& bRet) // no logging
{
   if (hSerial == RCM::INVALID_HANDLE) return false;

   DWORD cbRead;
   if (!ReadFile(hSerial, &bRet, 1, &cbRead, NULL))
   {
      //printDump( "FATAL: ReadFile failed for serial port\n");
      //exit(-1);
		return false;
   }
   if (cbRead == 1)
      return true;
   return false;   // no data
}

static bool ReadSerialBytes(HANDLE hSerial, void* pb, int cb)
{
   if (hSerial == RCM::INVALID_HANDLE) return false;
   DWORD cbRead;
   if (!ReadFile(hSerial, pb, cb, &cbRead, NULL) || (int)cbRead != cb)
      return false;
   return true;
}

static bool SendSerialBytes(HANDLE hSerial, const void* pb, int cb)
{
   if (hSerial == RCM::INVALID_HANDLE) return false;
   DWORD cbWrite;
   if (!WriteFile(hSerial, pb, cb, &cbWrite, NULL) || (int)cbWrite != cb)
      return false;
   return true;
}


///////////////////////////////////////////////////////////
// RCM::RCM

RCM::RCM()
{
   if (tmpLine == NULL) {
      tmpLine = new char[1024];
   }
   m_hSerial = RCM::INVALID_HANDLE;
   m_nErrorCount = 0;
   MaxVelocity = 700000;
   MaxAcceleration = 176;
   status = false;
   Powers[0]=0;
   Powers[1]=0;
   debug_last_opcode = 0;
   DigitalOutputValue = 0;
   connectedFlag = false;
}

RCM::~RCM()
{
   close();
}

void RCM::Purge()//(const char* szTag)
{
   if (m_hSerial == RCM::INVALID_HANDLE) return;
   //BYTE bIgnore;
   //while (PeekSerialByte(m_hSerial, bIgnore)) {};
   //printf("[purge %s$%x] ", szTag, bIgnore);
	do {
		int available;
		ioctl(m_hSerial, FIONREAD, &available);
		if (available > 0) {
			char buffer[available];
			if (read(m_hSerial, buffer, available) < 1) {
				fprintf(stderr, "Purge read error\n");
			}
		} else {
			break;
		}
	} while (true);
   m_nErrorCount = 0;
}

void RCM::close()
{
   initialized = false;
   if (m_hSerial == RCM::INVALID_HANDLE) return;
   MotorsOff(); //stop motors
   if (m_hSerial != RCM::INVALID_HANDLE)
      closeHandle(m_hSerial);
   m_hSerial = RCM::INVALID_HANDLE;
}

bool RCM::Open(char * iCommPort)
{
   status = false;
   initialized = false;
   //assert(m_hSerial == RCM::INVALID_HANDLE);
   if (m_hSerial != RCM::INVALID_HANDLE) {
   std::cout << "FATAL:Handle\n";

      close();
   }

   //  LastConnectionAttempt = GetTickCount();
   m_hSerial = OpenSerial(iCommPort);
   if (m_hSerial == RCM::INVALID_HANDLE)
   {
        std::cout << "FATAL: COM port is busy\n";
      //PRINT_VERBOSE(("FATAL: COM port is busy\n"));
      return false;
   }
   if (!setCommDefaults(iCommPort))
   {
      //PRINT_VERBOSE(("FATAL: setCommDefaults error\n"));
      //close();
      std::cout << "FATAL: setCommDefaults error\n";
      return false;
   }


 /*  char zero = 0;
   char read = 0;
   fprintf(stdout, "START SENDING ZEROS");
   while (1) {
      SendSerialBytes(m_hSerial, &zero, 1);
      if (ReadSerialBytes(m_hSerial, &read, 1)) {
	 fprintf(stdout, "Readed %d\n", read);
      }

   }*/


   // RCM has two motion processors
   int nGood = 0;
   for (int iPimp = 0; iPimp < NUM_PIMP; iPimp++)
   {
      Purge();
      //PRINT_VERBOSE(("\tprobing Motion Processor#%d -- ", iPimp));

      if (Send0(iPimp, OP_GetChecksum))
      {
	 //	 Sleep(50);
	 ulong lCheck;
	 if (!Recv32(iPimp, lCheck))
	 {
	    printf( "not responding");
	 }
	 else if (lCheck != 0x12345678)
	 {
	    printf( "error bad checksum $%lx", lCheck);
	 }
	 else
	 {
	    nGood++;
	 }
      }
      else
      {
	 //printf("send error\n");
      }
   }

   if (nGood != NUM_PIMP)
   {
      printf("FATAL: can't find a working RCM on that COM port\n");

      //close();
      return false;
   }

   //printDump( "InitDefaults");
   if (!InitDefaults()) return false;

   status = true;
   setConnected(true);
   return true;
}

bool RCM::Recover()
{
   if (m_hSerial == RCM::INVALID_HANDLE) return false;
   Purge();
   int i=100;
   do{
      char b = 0;
      if (SendSerialBytes(m_hSerial, &b, 1));
      //Sleep(50);
      if (ReadSerialBytes(m_hSerial, &b, 1)){
	 if (b==0){
	    Purge();
	    return true;
	 }
      }
      i--;
   }while(i>0);
   return false;
}

///////////////////////////////////////////////////////////////

bool RCM::Send0(int iPimp, byte opcode)
{
   if (m_hSerial == RCM::INVALID_HANDLE) return false;
   debug_last_opcode = opcode;
   PIMP_COMMAND pimpc;
   pimpc.address = iPimp;
   pimpc.axis = 0;
   pimpc.opcode = opcode;
   int cb = pimpc.UpdateChecksum(0);

   if (!SendSerialBytes(m_hSerial, &pimpc, cb))
   {
      //PRINT_VERBOSE(("ERROR: RCM::Send failed $%x\n", opcode));
      m_nErrorCount++;
      status = false;
      return false;
   }
   status = true;
   return true;
}

bool RCM::Ack()
{
   if (m_hSerial == RCM::INVALID_HANDLE) return false;
   //Sleep(REVIEW_SLEEP); // REVIEW

   byte rgb[2];
   //if (!ReadSerialBytes(m_hSerial, rgb, 2) || rgb[0] != 0 || rgb[1] != 0)
   long t = GetTickCount();
   while (!ReadSerialBytes(m_hSerial, rgb, 2)){
      if (GetTickCount()>=t+READ_TIMEOUT){
			rgb[0]=1; //signal error
			break;
      }
   }
   if (rgb[0] != 0 || rgb[1] != 0)
   {
      //PRINT_VERBOSE(("ERROR: RCM::Ack failed\n"));
      m_nErrorCount++;
      status = false;
      return false;
   }
   status = true;
   return true;
}

bool RCM::Ack32()
{
   if (m_hSerial == RCM::INVALID_HANDLE) return false;
   //Sleep(REVIEW_SLEEP); // REVIEW

   byte rgb[4];
   //if (!ReadSerialBytes(m_hSerial, rgb, 4) || rgb[0] != 0 || rgb[1] != 0 || rgb[2] != 0 || rgb[3] != 0)
   long t = GetTickCount();
   while (!ReadSerialBytes(m_hSerial, rgb, 4)){
      if (GetTickCount()>=t+READ_TIMEOUT){
	 rgb[0]=1; //signal error
	 break;
      }
   }
   if (rgb[0] != 0 || rgb[1] != 0 || rgb[2] != 0 || rgb[3] != 0)
   {
      //PRINT_VERBOSE(("ERROR: RCM::Ack32 failed\n"));
      m_nErrorCount++;
      status = false;
      return false;
   }
   status = true;
   return true;
}

bool RCM::Send16(int iPimp, byte opcode, uword wArg)
{
   if (m_hSerial == RCM::INVALID_HANDLE) return false;
   debug_last_opcode = opcode;
   PIMP_COMMAND pimpc;
   pimpc.address = iPimp;
   pimpc.axis = 0;
   pimpc.opcode = opcode;
   pimpc.data[0] = HIBYTE(wArg);
   pimpc.data[1] = LOBYTE(wArg);
   int cb = pimpc.UpdateChecksum(2);

   if (!SendSerialBytes(m_hSerial, &pimpc, cb))
   {
      //PRINT_VERBOSE(("ERROR: RCM::Send16 failed $%x\n", opcode));
      m_nErrorCount++;
      status = false;
      return false;
   }
   status = true;
   return true;
}

bool RCM::Send32(int iPimp, byte opcode, ulong lArg)
{
   if (m_hSerial == RCM::INVALID_HANDLE) return false;
   debug_last_opcode = opcode;
   PIMP_COMMAND pimpc;
   pimpc.address = iPimp;
   pimpc.axis = 0;
   pimpc.opcode = opcode;
   pimpc.data[0] = HIBYTE(HIWORD(lArg));
   pimpc.data[1] = LOBYTE(HIWORD(lArg));
   pimpc.data[2] = HIBYTE(LOWORD(lArg));
   pimpc.data[3] = LOBYTE(LOWORD(lArg));
   int cb = pimpc.UpdateChecksum(4);

   if (!SendSerialBytes(m_hSerial, &pimpc, cb))
   {
      //PRINT_VERBOSE(("ERROR: RCM::Send32 failed $%x\n", opcode));
      m_nErrorCount++;
      status = false;
      return false;
   }
   status = true;
   return true;
}

bool RCM::Recv16(int iPimp, uword& wRet)
{
   if (m_hSerial == RCM::INVALID_HANDLE) return false;
   PIMP_RESPONSE pimpr;
   //int cb = PIMP_RESPONSE_SIZE(2);
   //if (!ReadSerialBytes(m_hSerial, &pimpr, PIMP_RESPONSE_SIZE(2)) || !pimpr.CheckChecksum(2) || pimpr.status != 0)
   long t = GetTickCount();
   while (!ReadSerialBytes(m_hSerial, &pimpr, PIMP_RESPONSE_SIZE(2))){
      if (GetTickCount()>=t+READ_TIMEOUT){
	 pimpr.status=1; //signal error
	 break;
      }
   }
   if (!pimpr.CheckChecksum(2) || pimpr.status != 0)
   {
      //PRINT_VERBOSE(("ERROR: RCM::Recv16 failed\n"));
      m_nErrorCount++;
      status = false;
      return false;
   }
   wRet = MAKEWORD(pimpr.data[1], pimpr.data[0]);
   status = true;
   return true;
}

bool RCM::Recv32(int iPimp, ulong& lRet)
{
   if (m_hSerial == RCM::INVALID_HANDLE) return false;
   PIMP_RESPONSE pimpr;
   //int cb = PIMP_RESPONSE_SIZE(4);
   //if (!ReadSerialBytes(m_hSerial, &pimpr, PIMP_RESPONSE_SIZE(4)) || !pimpr.CheckChecksum(4) || pimpr.status != 0)
   long t = GetTickCount();
   while (!ReadSerialBytes(m_hSerial, &pimpr, PIMP_RESPONSE_SIZE(4))) {
      if (GetTickCount()>=t+READ_TIMEOUT){
	 pimpr.status=1; //signal error
	 break;
      }
   }
   if (!pimpr.CheckChecksum(4) || pimpr.status != 0)
   {
      //PRINT_VERBOSE(("ERROR: RCM::Recv32 failed\n"));
      m_nErrorCount++;
      status = false;
      return false;
   }
   WORD w1 = MAKEWORD(pimpr.data[1], pimpr.data[0]);
   WORD w2 = MAKEWORD(pimpr.data[3], pimpr.data[2]);
   lRet = MAKELONG(w2, w1);
   status = true;
   return true;
}

///////////////////////////////////////////////////////////////

bool RCM::Reset()
{
   initialized = false;
   for (int i=1; i>=0; i--){
      if (!Send0(i, OP_GetVersion)) return false;
      //Sleep(50);
      unsigned long Version;
      if (!Recv32(i, Version)) return false;
      if (!Send0(i, OP_Reset)) return false;
      Sleep(200);
      Purge();
      if (!Send0(i, OP_GetHostIOError)) return false;
      //Sleep(50);
      uword wValue;
      if (!Recv16(i, wValue)) return false;
      //if (wValue!=1) return false; GetHostIOError should return 1 after restart
   }
   return InitDefaults();
}

bool RCM::InitDefaults()
{
   for (int iPimp = 1; iPimp >= 0; iPimp--)
   {
      /*
	 Nenastavovat nic - pouzije se defaultni nastaveni
	 if (!Send16(iPimp, OP_setOutputMode, 1) || !Ack()) // PWM mode (signed)
	 return false;
	 if (!Send16(iPimp, OP_setEncoderSource, 2) || !Ack()) // no encoder
	 return false;
	 if (!Send16(iPimp, OP_setPhaseCounts, 256) || !Ack()) // 256 steps / revolution
	 return false;
	 */
 //     printDump( "send OP_setLimitSwitchMode\n");
      if (!Send16(iPimp, OP_setLimitSwitchMode, 0) || !Ack()) // No limit switches
	 return false;
      if (!Send16(iPimp, OP_setMotorCommand, 0) || !Ack()) // Zero power to the motors
	 return false;
      // GetSignalStatus - zatim nepotrebuji
      if (!Send16(iPimp, OP_setProfileMode, 1) || !Ack()) //Velocity Contouring
	 return false;
      //if (!Send32(iPimp, OP_setPositionErrorLimit, 0x7FFFFFFF) || !Ack32())
      //    return false;
      if (!Send32(iPimp, OP_setAcceleration, 176) || !Ack())
	 return false;
      if (!Send32(iPimp, OP_setVelocity, 0) || !Ack())
	 return false;
   }
   if (!Send0(1, OP_Update) || !Ack()) return false;
   if (!Send0(0, OP_Update) || !Ack()) return false;
   DigitalOutputValue = 0;
   WriteDigitalValue(0, DigitalOutputValue);
   initialized = true;
   setConnected(true);
   return true;
}

bool RCM::Check()
{
   if (!Connected()) return false;
   for (int iPimp = 0; iPimp < NUM_PIMP; iPimp++){
      Purge();
      if (Send0(iPimp, OP_GetChecksum)){
	 //Sleep(50);
	 ulong lCheck;
	 if (!Recv32(iPimp, lCheck)) return false;
	 else if (lCheck != 0x12345678) return false;
      }else return false;
   }
   return true;
}

bool RCM::MotorsOnOff(uword wValue)
{
   for (int iPimp = 0; iPimp < NUM_PIMP; iPimp++)
   {
      //if (!Send16(iPimp, OP_setMotorMode, wValue) || !Ack()) return false;
      if (wValue==1){
	 //if (!Send16(iPimp, OP_setOutputMode, 1) || !Ack()) return false; //signed PWM - default
	 //if (!Send16(iPimp, OP_setProfileMode, 1) || !Ack()) return false; //velocity contouring - is set elsewhere
	 //if (!Send0(iPimp, OP_Update) || !Ack()) return false;
      }else{
	 if (!Send16(iPimp, OP_setMotorCommand, 0) || !Ack()) return false; //zero power
	 if (!Send0(iPimp, OP_Update) || !Ack()) return false;
      }
   }
   return true;
}

///////////////////////////////////////////////////////////////

WORD RCM::ReadAnalogValue(int iPimp, int iPort)
{
   if (!Send16(iPimp, OP_ReadAnalog, (uword)iPort)) return 0xFFFF;
   //Sleep(REVIEW_SLEEP); // REVIEW
   uword wValue;
   if (!Recv16(iPimp, wValue)) return 0xFFFF;
   return wValue;
}

WORD RCM::ReadDigitalValue(int address)
{
   // ER1 only 1 digital port
   //assert(iPimp == 0); // ER1
   int iPimp = 0;
   assert(address == 0); // ER1

   if (!Send16(iPimp, OP_ReadIO, (uword)address)) return 0xFFFF;
   //Sleep(REVIEW_SLEEP); // REVIEW
   uword wValue;
   if (!Recv16(iPimp, wValue)) return 0xFFFF;
   return wValue;
}

bool RCM::WriteDigitalValue(int address, WORD wData)
{
   // ER1 only 1 digital port
   //assert(iPimp == 0); // ER1
   /*   int iPimp = 0;
	assert(address == 0); // ER1
	assert(wData < 256);    // just a byte
   //REVIEW: really Send1616
   if (!Send32(iPimp, OP_WriteIO, MAKELONG(wData, address))) return false;
   if (!Ack()) return false;
   */
   return true;
}

bool RCM::setDigitalOutput(int bit, int value)
{
   bit=bit&0x7;
   if (value!=0){
      DigitalOutputValue |= (1<<bit);
   }else{
      DigitalOutputValue &= ~(1<<bit);
   }
   return WriteDigitalValue(0, DigitalOutputValue);
}

//==============================================================================

int RCM::GetValue(int iPimp, byte opcode)
{
   Purge();
   if (!Send0(iPimp, opcode)) return 0;
   //Sleep(REVIEW_SLEEP);

   //long value = -99999;
   //if (!Recv16(iPimp, wValue)) return false;
   ulong lValue;
   if (!Recv32(iPimp, lValue)) return 0;
   return lValue;
}

bool RCM::SafeGetValue(int iPimp, byte opcode, int &result)
{
   Purge();
   if (!Send0(iPimp, opcode)) return false;
   ulong lValue;
   if (!Recv32(iPimp, lValue)) return false;
   result = lValue;
   return true;
}

//==============================================================================
bool RCM::setPower(int iPimp, float power, bool update) //power=0-1
{
   //int pwr =power*0x7fff;
   int pwr = (int)(power*0x7fff);
   if (pwr>0x7fff) pwr=0x7fff;
   if (pwr<0) pwr=0;
   for (int i=1; i>=0; i--){
      if ((i==iPimp)||(iPimp==2)){
	 //if (pwr!=Powers[i]){
	 if (!Send16(i, OP_setMotorCommand, pwr) || !Ack()) {setConnected(false); return false;}
	 Powers[i]=pwr;
	 if (update){
	    if (!Send0(i, OP_Update) || !Ack()) {setConnected(false); return false;}
	 }
	 //}
      }
   }
   return true;
}

bool RCM::setVelocity(int iPimp, float velocity, bool update) //velocity=0-1
{
   //if (velocity>1.0) velocity=1.0;
   //if (velocity<-1.0) velocity=-1.0;
   //int iVel = velocity*MaxVelocity;
   int iVel = (int)(velocity*MaxVelocity);
   if (iVel>MAX_VELOCITY) iVel=MAX_VELOCITY;
   if (iVel<-MAX_VELOCITY) iVel=-MAX_VELOCITY;
   for (int i=1; i>=0; i--){
      if ((i==iPimp)||(iPimp==2)){
	 if (!Send32(i, OP_setVelocity, iVel) || !Ack()) {setConnected(false); return false;}
	 if (update){
	    if (!Send0(i, OP_Update) || !Ack()) {setConnected(false); return false;}
	 }
      }
   }
   return true;
}

bool RCM::setVelocities(float vel1, float vel2, bool update) //velocity=0-1
{
   //if (vel1>1.0) vel1=1.0; if (vel1<-1.0) vel1=-1.0;
   //if (vel2>1.0) vel2=1.0; if (vel2<-1.0) vel2=-1.0;
   //int iVel1 = vel1*MaxVelocity;
   int iVel1 = (int)(vel1*MaxVelocity);
   if (iVel1>MAX_VELOCITY) iVel1=MAX_VELOCITY;
   if (iVel1<-MAX_VELOCITY) iVel1=-MAX_VELOCITY;
   //int iVel2 = vel2*MaxVelocity;
   int iVel2 = (int)(vel2*MaxVelocity);
   if (iVel2>MAX_VELOCITY) iVel2=MAX_VELOCITY;
   if (iVel2<-MAX_VELOCITY) iVel2=-MAX_VELOCITY;

   //setPower(1,(vel2!=0)?0.9:0.001,true);
   //setPower(0,(vel1!=0)?0.9:0.001,true);
   /*
      Powers[0]=((vel1!=0)?0.9:0.001)*0x7fff;
      Powers[1]=((vel2!=0)?0.9:0.001)*0x7fff;
      if (!Send16(1, OP_setMotorCommand, Powers[1]) || !Ack()) return false;
      if (!Send16(0, OP_setMotorCommand, Powers[0]) || !Ack()) return false;
      */
   if (!Send32(1, OP_setVelocity, iVel2) || !Ack()) {setConnected(false); return false;}
   if (!Send32(0, OP_setVelocity, iVel1) || !Ack()) {setConnected(false); return false;}
   if (update){
      if (!Send0(1, OP_Update) || !Ack()) {setConnected(false); return false;}
      if (!Send0(0, OP_Update) || !Ack()) {setConnected(false); return false;}
   }
   return true;
}

bool RCM::setAcceleration(int iPimp, float acceleration) //acceleration=0-1
{
   for (int i=0; i<2; i++){
      if ((i==iPimp)||(iPimp==2)){
	 //if (!Send32(i, OP_setAcceleration, acceleration*MaxAcceleration) || !Ack()) return false;
	 if (!Send32(i, OP_setAcceleration, (long)acceleration*MaxAcceleration) || !Ack()) {setConnected(false); return false;}
	 if (!Send0(i, OP_Update) || !Ack()) {setConnected(false); return false;}
      }
   }
   return true;
}

bool RCM::setDeceleration(int iPimp, float deceleration) //deceleration=0-1
{
   for (int i=0; i<2; i++){
      if ((i==iPimp)||(iPimp==2)){
	 //if (!Send32(i, OP_setDeceleration, deceleration*MaxAcceleration) || !Ack()) return false;
	 if (!Send32(i, OP_setDeceleration, (long)deceleration*MaxAcceleration) || !Ack()) {setConnected(false); return false;}
	 if (!Send0(i, OP_Update) || !Ack()) {setConnected(false); return false;}
      }
   }
   return true;
}

void RCM::setMaxVelocity(int maxvelocity)
{
   MaxVelocity = maxvelocity;
}

void RCM::setMaxAcceleration(int maxacceleration)
{
   MaxAcceleration = maxacceleration;
}

float RCM::GetVelocity(int iPimp)
{
   Purge();
   if (!Send0(iPimp, OP_GetVelocity)) {setConnected(false); return 0;}
   ulong lValue;
   if (!Recv32(iPimp, lValue)) {setConnected(false); return 0;}
   return (int)lValue/MaxVelocity;
}

float RCM::GetActualVelocity(int iPimp)
{
   Purge();
   if (!Send0(iPimp, OP_GetActualVelocity)) {setConnected(false); return 0;}
   ulong lValue;
   if (!Recv32(iPimp, lValue)) {setConnected(false); return 0;}
   return (int)lValue/MaxVelocity;
}

float RCM::GetCommandedVelocity(int iPimp)
{
   Purge();
   if (!Send0(iPimp, OP_GetCommandedVelocity)) {setConnected(false); return 0;}
   ulong lValue;
   if (!Recv32(iPimp, lValue)) {setConnected(false); return 0;}
   return (int)lValue/MaxVelocity;
}

bool RCM::GetMotorStates(bool &m0, bool &m1)
{
   Purge();
   unsigned short Value;
   if (!Send0(0, OP_GetSignalStatus)) {setConnected(false); return false;}
   //Sleep(REVIEW_SLEEP);
   if (!Recv16(0, Value)) return false;
   m0 = ((Value & 0x10)==0);   // (Value & 0x40) ~ LEVY/PRAVY
   if (!Send0(1, OP_GetSignalStatus)) {setConnected(false); return false;}
   //Sleep(REVIEW_SLEEP);
   if (!Recv16(1, Value)) return false;
   m1 = ((Value & 0x10)==0);
   return true;
}

bool RCM::GetMotorStatus(int iPimp, bool &st)
{
   Purge();
   unsigned short Value;
   if (!Send0(iPimp, OP_GetSignalStatus)) {setConnected(false); return false;}
   //Sleep(REVIEW_SLEEP);
   if (!Recv16(iPimp, Value)) {setConnected(false); return false;}
   st = ((Value & 0x10)==0);
   return true;
}

//-----------------------------------------------------------------------------
bool RCM::connected(void) {
   return connectedFlag;
}
