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
	motor_reply_t reply;
	
	//if (motors_open())
	motors_open();
	//{

	if (argv[1][0]=='r')
	{
		if (motors_getStatus(&reply)) {
			printf("motors_getStatus: Error %d: %s\n", (int) errno, (char*) strerror(errno));
		} else {
			printf("HWESTOP %d\n", (int) reply.HWESTOP);
			printf("AUTOMAN %d\n", (int) reply.AUTOMAN);
			printf("PATHNAV %d\n", (int) reply.PATHNAV);
			printf("CURRENTLEFT %d\n", ( ((int)reply.CURRENTLEFT1)<<8) | ((int)reply.CURRENTLEFT2) );
			printf("CURRENTRIGHT %d\n", ( ((int)reply.CURRENTRIGHT1)<<8) | ((int)reply.CURRENTRIGHT2) );
			printf("LOGICBATT %d\n", ( ((int)reply.LOGICBATT1)<<8) | ((int)reply.LOGICBATT2) );
			printf("MOTORBATT %d\n", ( ((int)reply.MOTORBATT1)<<8) | ((int)reply.MOTORBATT2) );
			printf("LEFTMOTORSPEED %d\n", ( ((int)reply.LEFTMOTORSPEED1)<<8) | ((int)reply.LEFTMOTORSPEED2) );
			printf("RIGHTMOTORSPEED %d\n", ( ((int)reply.RIGHTMOTORSPEED1)<<8) | ((int)reply.RIGHTMOTORSPEED2) );
		}
	}
	if (argv[1][0]=='w')
	{
		char variableName = (char) atoi(argv[2]);
		char variableValue = (char) atoi(argv[3]);
		
		if (motors_setVar(variableName, variableValue)) {
			printf("motors_setVar: Error %d: %s\n", (int) errno, (char*) strerror(errno));
		}
	}

	motors_close();
	//}
}

