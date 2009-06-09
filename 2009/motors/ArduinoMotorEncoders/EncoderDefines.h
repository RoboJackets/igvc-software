#ifndef ENCODER_DEFINES_H_
#define ENCODER_DEFINES_H_

/* MOTOR ENCODER DEFINITIONS */
#define COUNTER_SCALER			(64)
#define F_CPU 16000000
#define COUNTER_RATE			(F_CPU/COUNTER_SCALER)
#define TOTAL_ENCODER_TICKS 		((double)512)
#define RAD_PER_ENCODER_TICK		((double)(2*M_PI)/TOTAL_ENCODER_TICKS)
#define LEFT_MOTOR_ENCODER_DIRECTION 	1
#define RIGHT_MOTOR_ENCODER_DIRECTION 	0

/* ROBOT PHYSICAL DEFINITIONS */  /*TODO: determine these values */
#define MOTOR_RATIO		((double)20)
#define WHEEL_RADIUS		((double)5 / (double)MOTOR_RATIO)
#define WHEEL_BASE		((double)28)


//type for packet storage
typedef struct
{
	header_t head;
	size_t len;
	byte * msg;
} packet_t;

/*
//Header types
typedef struct __attribute__((__packed__)) { long timestamp; long packetnum; byte cmd; byte size; } header_t;

// Arduino -> Laptop ID packet
//typedef struct __attribute__((__packed__)) { unsigned int timestamp; unsigned int packetnum; char command; } idpk_t;

// Arduino -> Laptop data packet
typedef struct  __attribute__((__packed__)) { int dl; int dr; unsigned long dt; } reply_dtick_t;

typedef struct  __attribute__((__packed__)) { int currentl; int currentr; unsigned long dt; } reply_current_t;
*/
#endif //ENCODER_DEFINES_H_
