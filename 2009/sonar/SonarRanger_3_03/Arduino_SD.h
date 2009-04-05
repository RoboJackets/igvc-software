#define ATMEGA168
#define TIMEOUT_LENGTH_MILLIS 1000
#define NUM_PK_STORE 10
#define NULL 0

#ifndef ARDUINOCMDS_HPP_
#define ARDUINOCMDS_HPP_
#define PACKET_ERROR_CMD 0xFF
//1 byte commands, laptop -> arduino
#define ARDUINO_GETSTATUS_CMD 'r'
#define ARDUINO_SETVAR_CMD 'w'
#define ARDUINO_ID_CMD 'i'
#define ARDUINO_RSND_PK_CMD 'p'
//1 byte commands/response arduino -> laptop
#define ARDUINO_ERROR_RESP 0xFF
#define ARDUINO_UNKOWN_CMD_RESP 0xFE
#define ARDUINO_GETSTATUS_RESP 'r'
#define ARDUINO_SETVAR_RESP 'w'
#define ARDUINO_ID_RESP 'i'
#define ARDUINO_RSND_PK_RESP 'p'
// MC ENUMS
// Enums for data settings
enum mc_opttype_t { MC_PUSHPULL = 0, MC_RET_T, MC_INTEROG_DL, MC_SETCLK, MC_RESENDPKT};//interogdl - miliseconds
enum mc_opt_t {MC_PUSH = 0, MC_PULL, MC_SEND_DTICK, MC_SEND_CURRENT};
//enum for general errors
enum error_t {DROPPED_PACKET, REQUESTED_PACKET_OUT_OF_RANGE};
//enum for sonar options
enum sd_optype_t{	SREAD_ALL,SPING_ALL,SREAD_IND,SPING_IND,SET_GAIN_ALL,SET_GAIN_IND,SET_MRANGE_ALL,SET_MRANGE_IND,
					SET_FREQ,SET_STEP_IND,SET_STEP_TOT,SET_ACTIVE_IND,SET_ECHONUM,GET_GAIN_IND,GET_MRANGE_IND,
					GET_ACTIVE_IND,GET_STEP_IND,GET_FREQ,GET_STEP_TOT,START_SONDEV, SET_AUTO_ALL};
#endif //ARDUINO_COMMANDS_HPP_

#ifndef DataPacketStructs_H_
#define DataPacketStructs_H_
//length of header
#define PACKET_HEADER_SIZE 14
typedef unsigned char byte;
//Packet Header
typedef struct __attribute__((__packed__)) { long timestamp_sec; long timestamp_usec; unsigned long packetnum; byte cmd; byte size; } header_t; //orig arduino - should be same as laptop
//Data types
//Error Frame
typedef struct __attribute__((__packed__)) { int errnum; byte * msg; } error_pk_t;
//Encoder Data
typedef struct __attribute__((__packed__)) { short dl; short dr; unsigned int dt;} encoder_reply_t;
typedef struct __attribute__((__packed__)) { int dl; int dr; unsigned long dt; } reply_dtick_t; // orig arduino
//Current Data
typedef struct __attribute__((__packed__)) { short il; short ir;} current_reply_t;
typedef struct  __attribute__((__packed__)) { int currentl; int currentr; unsigned long dt; } reply_current_t; //orig arduino
//Motor Speed Frame
typedef struct __attribute__((__packed__)) { char sl; char sr;} speed_set_t;
//Sonar Data Frame
typedef struct __attribute__((__packed__)) { short id; short range;} sonar_range_t; //range in milimeters
#endif //DataPacketStructs_H_

void readSerial(void);
void setVariable(byte num, byte val);
void sendStatus();
void serialPrintBytes(void *data, int numBytes);
void serialPrintDouble(double data, int precision);
unsigned int getTime();
void resetTime();
void genTimestamp(long * sec, long * usec);
void sendPacket(char cmd, int dataSize, byte* data);
