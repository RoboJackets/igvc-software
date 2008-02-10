#ifndef _TYPES_H_
#define _TYPES_H_

/* Types defined in avr-libc */
#include <stdint.h>

/* Defines way the robot is controlled */
typedef enum{
	DRIVE_MODE = 0, /* only used for safteyMode */
	AUTONOMOUS_MODE = 0,
	NAVIGATION_MODE = 1,
	JOYSTICK_MODE = 2,
	STOP_MODE = 3 /* only used for safteyMode and communicating with the computer */
} CONTROLMODE_T;

/* Values used to control the motors */
typedef struct _motorstates_
{
	uint8_t leftSpeed, rightSpeed;	// 0 to 255
	uint8_t leftDir, rightDir;	// FORWARD or BACKWARD
} MOTORSTATES_T;

/* Provides access to the bits in a byte. */
typedef struct
{
	uint8_t bit_0:1;
	uint8_t bit_1:1;
	uint8_t bit_2:1;
	uint8_t bit_3:1;
	uint8_t bit_4:1;
	uint8_t bit_5:1;
	uint8_t bit_6:1;
	uint8_t bit_7:1;
} __bits;

/* Provides a _bits structure at the given address. */
#define IO_BYTE(addr)	(*(volatile __bits *)addr)

#endif /* _TYPES_H_ */
