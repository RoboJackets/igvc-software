#include <stdlib.h>
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <stdbool.h>
#include <cstdio>    /* Standard input/output definitions */
#include <errno.h>    /* Error number definitions */
#include <time.h>
#include <cmath>

#include "arduino_comm.h"

#define SERIALPORT "/dev/ttyUSB0"
#define BAUD B9600

//#define DT 100

void readToNewline(int fd, char * buff, int maxlen){
	int i = 0;
	do{
		readFully(fd, &buff[i], 1);
		i++;
	}while((strncmp(&buff[i],"\n",1) != 0) && (i < maxlen));
}

int main(void){
	char buffer[6] = {0};
	//char char_buff;
	//char a = 'a';
	int fd = serialport_init(SERIALPORT, BAUD);
	
	int dp;
	int dt;
	//struct timezone *tz;
	float velocity;
	
	while(1){
				
		readToNewline(fd, buffer, 2);
		if( strncmp(buffer,"dp",2) == 0){
			readToNewline(fd, buffer, 5);
			dp = atoi(buffer);
		}
		else if( strncmp(buffer,"dt",2) == 0){
			readToNewline(fd, buffer, 5);
			dt = atoi(buffer);
		}		
		else{
			printf("broke");
			readFully(fd, buffer, 1);
		}

		readToNewline(fd, buffer, 2);
		if( strncmp(buffer,"dp",2) == 0 ){
			readToNewline(fd, buffer, 5);
			dp = atoi(buffer);
		}
		else if( strncmp(buffer,"dt",2) == 0){
			readToNewline(fd, buffer, 5);
			dt = atoi(buffer);
		}		
		else{
			printf("broke");
			readFully(fd, buffer, 1);
		}

		velocity = (dp) * ((2* M_PI)/512) / dt*1000;
		printf("dp: %d\tdt: %d\tvel: %d\n", dp, dt, velocity);

		
		/*
		readFully(fd, buffer, 5);
		printf("%s\n",buffer);
		*/
	}
	return(0);
}

