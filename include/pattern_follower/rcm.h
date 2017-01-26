/*
 * File name: rcm.h
 * Date:      2005/02/10 15:33
 * Author:    Jan Faigl
 */

#ifndef __RCM_H__
#define __RCM_H__
#include <assert.h>
#include <iostream>
typedef unsigned char byte;
typedef unsigned short uword;
typedef unsigned short WORD;
typedef unsigned long ulong;
/*----------------------------------------------------------------------------
 * compatible define
 *----------------------------------------------------------------------------*/
#define max(x, y) ((x) > (y)) ? (x) : (y)
#define HANDLE int
#define BYTE byte
#define DWORD int

/*----------------------------------------------------------------------------
 * enum types
 *----------------------------------------------------------------------------*/
const int NUM_PIMP = 2;
#define MAX_VELOCITY 1500000

#define REVIEW_SLEEP 20
#define READ_TIMEOUT 50

#define MAX_CHARS 4096
//#define STTY_PATTERN "stty -F %s 230400 cs8 -parenb -parodd -cstopb"
//#define STTY_PATTERN "stty -F %s
//10:0:18b3:0:3:1c:7f:15:4:0:0:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0"
#define STTY_PATTERN                                                           \
  "stty -F %s 230400 -parenb -parodd cs8 -hupcl -cstopb cread clocal "         \
  "-crtscts -ignbrk -brkint -ignpar -parmrk inpck -istrip -inlcr -igncr "      \
  "-icrnl -ixon -ixoff  -iuclc -ixany -imaxbel -opost -olcuc -ocrnl -onlcr "   \
  "-onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0   ff0 -isig -icanon "     \
  "-iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop  -echoprt "      \
  "-echoctl -echoke"

/*----------------------------------------------------------------------------
 * enum types
 *----------------------------------------------------------------------------*/

typedef enum {
  OP_NoOperation = 0x00,
  //	OP_setMotorLimit = 0x06, // servo only
  //	OP_GetMotorLimit = 0x07,
  OP_setMotorBias = 0x0F,
  OP_setPosition = 0x10,
  OP_setVelocity = 0x11,
  OP_setJerk = 0x13,
  OP_setGearRatio = 0x14,
  OP_Update = 0x1A,
  OP_GetCommandedPosition = 0x1D,
  OP_GetCommandedVelocity = 0x1E,
  //	OP_setKp = 0x25,
  //	OP_setKi = 0x26,
  //	OP_setKd = 0x27,
  //	OP_setKvff = 0x2B,
  OP_GetPhaseAngle = 0x2C,
  //	OP_GetMotorBias = 0x2D,
  OP_setInterruptMask = 0x2F,
  OP_GetEventStatus = 0x31,
  OP_ResetEventStatus = 0x34,
  OP_GetCaptureValue = 0x36,
  OP_GetActualPosition = 0x37,
  OP_setSampleTime = 0x38,
  OP_Reset = 0x39,
  OP_GetCurrentMotorCommand = 0x3A,
  OP_GetTime = 0x3E,
  OP_ClearPositionError = 0x47,
  OP_GetPosition = 0x4A,
  OP_GetVelocity = 0x4B,
  OP_GetAcceleration = 0x4C,
  OP_setActualPosition = 0x4D,
  //	OP_GetKp = 0x50,//servo only
  //	OP_GetKi = 0x51,
  //	OP_GetKd = 0x52,
  //	OP_GetKvff = 0x54,//servo only
  OP_GetInterruptMask = 0x56,
  OP_GetJerk = 0x58,
  OP_GetGearRatio = 0x59,
  OP_GetSampleTime = 0x61,
  OP_GetMotorCommand = 0x69,
  OP_setStartVelocity = 0x6A,
  OP_GetStartVelocity = 0x6B,
  OP_GetOutputMode = 0x6E,
  //	OP_setPhaseInitializeTime = 0x72,
  OP_setPhaseCounts = 0x75,
  //	OP_setPhaseOffset = 0x76,
  OP_setMotorCommand = 0x77,
  //	OP_InitializePhase = 0x7A, -- 3310 only
  //	OP_GetPhaseOffset = 0x7B,
  //	OP_GetPhaseInitializeTime = 0x7C,
  OP_GetPhaseCounts = 0x7D,
  OP_setLimitSwitchMode = 0x80,
  OP_GetLimitSwitchMode = 0x81,
  OP_WriteIO = 0x82,
  OP_ReadIO = 0x83,
  OP_setPhaseAngle = 0x84,
  OP_setNumberPhases = 0x85,
  OP_GetNumberPhases = 0x86,
  OP_setAxisMode = 0x87,
  OP_GetAxisMode = 0x88,
  OP_setDiagnosticPortMode = 0x89,
  OP_GetDiagnosticPortMode = 0x8A,
  OP_setSerialPortMode = 0x8B,
  OP_GetSerialPortMode = 0x8C,
  OP_setEncoderModulus = 0x8D,
  OP_GetEncoderModulus = 0x8E,
  OP_GetVersion = 0x8F,
  OP_setAcceleration = 0x90,
  OP_setDeceleration = 0x91,
  OP_GetDeceleration = 0x92,
  //	OP_setKaff = 0x93,
  //	OP_GetKaff = 0x94,
  //	OP_setIntegrationLimit = 0x95,
  //	OP_GetIntegrationLimit = 0x96,
  OP_setPositionErrorLimit = 0x97,
  OP_GetPositionErrorLimit = 0x98,
  OP_GetPositionError = 0x99,
  //	OP_GetIntegral = 0x9A,
  //	OP_GetDerivative = 0x9B,
  //	OP_setDerivativeTime = 0x9C,
  //	OP_GetDerivativeTime = 0x9D,
  //	OP_setKout = 0x9E,
  //	OP_GetKout = 0x9F,
  OP_setProfileMode = 0xA0,
  OP_GetProfileMode = 0xA1,
  OP_setSignalSense = 0xA2,
  OP_GetSignalSense = 0xA3,
  OP_GetSignalStatus = 0xA4,
  OP_GetHostIOError = 0xA5,
  OP_GetActivityStatus = 0xA6,
  OP_GetCommandedAcceleration = 0xA7,
  OP_setTrackingWindow = 0xA8,
  OP_GetTrackingWindow = 0xA9,
  OP_setsettleTime = 0xAA,
  OP_GetsettleTime = 0xAB,
  OP_ClearInterrupt = 0xAC,
  OP_GetActualVelocity = 0xAD,
  OP_setGearMaster = 0xAE,
  OP_GetGearMaster = 0xAF,
  OP_setTraceMode = 0xB0,
  OP_GetTraceMode = 0xB1,
  OP_setTraceStart = 0xB2,
  OP_GetTraceStart = 0xB3,
  OP_setTraceStop = 0xB4,
  OP_GetTraceStop = 0xB5,
  OP_setTraceVariable = 0xB6,
  OP_GetTraceVariable = 0xB7,
  OP_setTracePeriod = 0xB8,
  OP_GetTracePeriod = 0xB9,
  OP_GetTraceStatus = 0xBA,
  OP_GetTraceCount = 0xBB,
  OP_setsettleWindow = 0xBC,
  OP_GetsettleWindow = 0xBD,
  OP_setActualPositionUnits = 0xBE,
  OP_GetActualPositionUnits = 0xBF,
  OP_setBufferStart = 0xC0,
  OP_GetBufferStart = 0xC1,
  OP_setBufferLength = 0xC2,
  OP_GetBufferLength = 0xC3,
  OP_setBufferWriteIndex = 0xC4,
  OP_GetBufferWriteIndex = 0xC5,
  OP_setBufferReadIndex = 0xC6,
  OP_GetBufferReadIndex = 0xC7,
  OP_WriteBuffer = 0xC8,
  OP_ReadBuffer = 0xC9,
  OP_setBufferFunction = 0xCA,
  OP_GetBufferFunction = 0xCB,
  OP_setStartMode = 0xCC,
  OP_setStopMode = 0xD0,
  OP_GetStopMode = 0xD1,
  OP_setAutoStopMode = 0xD2,
  OP_GetAutoStopMode = 0xD3,
  OP_setBreakpoint = 0xD4,
  OP_GetBreakpoint = 0xD5,
  OP_setBreakpointValue = 0xD6,
  OP_GetBreakpointValue = 0xD7,
  OP_setCaptureSource = 0xD8,
  OP_GetCaptureSource = 0xD9,
  OP_setEncoderSource = 0xDA,
  OP_GetEncoderSource = 0xDB,
  OP_setMotorMode = 0xDC,
  OP_GetMotorMode = 0xDD,
  OP_setEncoderToStepRatio = 0xDE,
  OP_GetEncoderToStepRatio = 0xDF,
  OP_setOutputMode = 0xE0,
  OP_GetInterruptAxis = 0xE1,
  //	OP_setCommutationMode = 0xE2,
  //	OP_GetCommutationMode = 0xE3,
  //	OP_setPhaseInitializeMode = 0xE4,
  //	OP_GetPhaseInitializeMode = 0xE5,
  //	OP_setPhasePrescale = 0xE6,
  //	OP_GetPhasePrescale = 0xE7,
  //	OP_setPhaseCorrectionMode = 0xE8,
  //	OP_GetPhaseCorrectionMode = 0xE9,
  OP_GetPhaseCommand = 0xEA,
  OP_setMotionCompleteMode = 0xEB,
  OP_GetMotionCompleteMode = 0xEC,
  OP_setAxisOutSource = 0xED,
  OP_GetAxisOutSource = 0xEE,
  OP_ReadAnalog = 0xEF,
  //	OP_setSynchronizationMode = 0xF2,
  //	OP_GetSynchronizationMode = 0xF3,
  OP_AdjustActualPosition = 0xF5,
  OP_GetChecksum = 0xF8
} PIMP_OP;

/*----------------------------------------------------------------------------
 * Class RCM
 *----------------------------------------------------------------------------*/
class RCM {
public:
  static HANDLE const INVALID_HANDLE;
  RCM(void);
  bool Open(char *iCommPort); // open serial and check PIMPs
  void close();
  ~RCM();
  int LastConnectionAttempt;
  bool initialized;
  WORD DigitalOutputValue;

protected:
  int m_hSerial;
  int m_nErrorCount;
  bool status;
  int Powers[2];
  int MaxVelocity;
  int MaxAcceleration;

  byte debug_last_opcode;
  // just a virtual serial port to the RCM
  //
public:
  // communication
  int GetErrorCount() { return m_nErrorCount; }
  // void Purge(const char* szTag = "");   // purge input and reset error count
  void Purge(); // purge input and reset error count
  bool Recover();
  bool Connected() { return m_hSerial != INVALID_HANDLE; };

  // no ack
  bool Send0(int iPimp, byte opcode);
  bool Send16(int iPimp, byte opcode, uword wArg);
  // with two byte ack
  bool Send32(int iPimp, byte opcode, ulong lArg);
  // with two byte ack

  bool Ack(); // Ack16
  bool Ack32();

  bool Recv16(int iPimp, uword &wRet);
  bool Recv32(int iPimp, ulong &lRet);

public:
  // ER1 specifics
  bool Reset();
  bool InitDefaults();
  bool Check();
  bool MotorsOn() { return MotorsOnOff(1); }
  bool MotorsOff() { return MotorsOnOff(0); }

  WORD ReadAnalogValue(int iPimp, int iPort);
  WORD ReadDigitalValue(int address);
  bool WriteDigitalValue(int address, WORD wData);

  bool setDigitalOutput(int bit, int value);

protected:
  bool MotorsOnOff(uword wValue);

public:
  int GetValue(int iPimp, byte opcode);
  bool SafeGetValue(int iPimp, byte opcode, int &result);

  bool setPower(int iPimp, float power, bool update = true); // power=0-1
  bool setVelocity(int iPimp, float velocity,
                   bool update = true); // velocity=0-1
  bool setVelocities(float vel1, float vel2, bool update = true); // velocity=0-1
  bool setAcceleration(int iPimp, float acceleration);       // acceleration=0-1
  bool setDeceleration(int iPimp, float deceleration = 0.0); // deceleration=0-1
  void setMaxVelocity(int maxvelocity);
  void setMaxAcceleration(int maxacceleration);
  float GetVelocity(int iPimp);
  float GetActualVelocity(int iPimp);
  float GetCommandedVelocity(int iPimp);
  int GetPosition(int iPimp) { return GetValue(iPimp, OP_GetPosition); };
  int GetActualPosition(int iPimp) {
    return GetValue(iPimp, OP_GetActualPosition);
  };
  int GetCommandedPosition(int iPimp) {
    return GetValue(iPimp, OP_GetCommandedPosition);
  };
  int GetCommandedPositionSafe(int iPimp, int &result) {
    return SafeGetValue(iPimp, OP_GetCommandedPosition, result);
  };
  bool GetStatus() {
    if (status)
      return true;
    else
      return Check();
  };
  int GetCurrentPower() { return max(Powers[0], Powers[1]); };
  bool GetMotorStates(bool &m0, bool &m1);  // identifies disconnected motor
  bool GetMotorStatus(int iPimp, bool &st); // identifies disconnected motor

private:
  inline void setConnected(bool connected) { connectedFlag = connected; };
  bool connectedFlag;

public:
  /**
   * return true if rcm is connected otherwise false
   *
   */
  bool connected(void);
};

struct PIMP_COMMAND {
  byte address; // 0 or 1 to select motor
  byte checksum;
  byte axis;    // lower 4 bits (always zero in ER1 case)
  byte opcode;  // see PIMP_OP below
  byte data[6]; // 0->6 bytes

  int UpdateChecksum(int cbData);
};

inline int PIMP_COMMAND_SIZE(int cbData) { return 1 + 1 + 1 + 1 + cbData; }

struct PIMP_RESPONSE {
  byte status;
  byte checksum;
  byte data[6]; // 0->6 bytes

  bool CheckChecksum(int cbData);
};

inline int PIMP_RESPONSE_SIZE(int cbData) { return 1 + 1 + cbData; }

//#include <poppack.h>

enum PIMP_OP_Profile {
  OP_PM_Trapezoidal = 0,
  OP_PM_Velocity = 1,
  OP_PM_SCurve = 2,
  OP_PM_External = 4,
};

#endif
/* end of rcm.h */
