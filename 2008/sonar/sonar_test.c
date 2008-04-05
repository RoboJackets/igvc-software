/*
 * When executing the following line at the command line prompt, an 'r' is sent over the serial connection to the arduino and the arduino dumps its data back in response
 * 
 *     motors_test r
 * 
 * In order to write to a variable on the arduino, type the following line into the command prompt.
 * Replace #1 with the number of the variable that you wish to write to, and replace #2 with the
 * value you wish to write to that variable.
 * 
 *     motors_test w #1 #2
 * 
 */
 //variable 0 is left motor speed
 //variable 1 is right motor speed
 //variable 2 is soft estop

#include <stdio.h>    /* Standard input/output definitions */
#include <errno.h>    /* Error number definitions */

#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <stdbool.h>
#include "motors.h"

int main(int argc, char *argv[]) {
	unsigned char status[22];
	
	sonar_open();
	sonar_setMode(SIMULTANEOUS);
	if (sonar_getStatus(status)) {
		printf("sonar_getStatus: Error %d: %s\n", (int) errno, (char*) strerror(errno));
	} else {
		printf("sensor0: %d\n", (int) status[0]);
	}
	sonar_close();
	//}
}

