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
