#ifndef MOTORS_H
#define MOTORS_H

#include <stdlib.h>
#include <termios.h>  // for speed_t
#include <stdbool.h>  // for bool
extern unsigned char status[22]
typedef enum {
	//read the sensors from left to right
	LEFT_TO_RIGHT	= 2,
	//read the sensors from right to left
	RIGHT_TO_LEFT	= 3,
	//read the sensors all at once
	SIMULTANEOUS	= 1,
} sonar_mode_t;

bool sonar_open(void);
void sonar_close(void);
bool sonar_getStatus(unsigned char *status);
bool sonar_setMode(sonar_mode_t mode);

#endif /* MOTORS_H */

