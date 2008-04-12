#include <cstdlib>
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
//#include <cmath>
//#include <iostream>
#include <unistd.h>

#include <assert.h>

#include "arduino_comm.h"

#define SERIALPORT "/dev/ttyUSB0"
#define BAUD B9600

void readToNewline(int fd, char * buff, int maxlen){
	int i = 0;
	do{
		readFully(fd, &buff[i], 1);
		i++;
	}while((strncmp(&buff[i],"\n",1) != 0) && (i < maxlen));
	buff[i] = '\0';
}

int main(void){
	char buffer[10] = {0};
	int arduino_fd = serialport_init(SERIALPORT, BAUD);
	assert(arduino_fd != -1);
	char p[2] = "p";
	
	while(1){
	if(writeFully(arduino_fd, p, 1)){
			//readToNewline(arduino_fd, buffer, 8);
			readFully(arduino_fd, buffer, 6);
			printf("rec %s\n", buffer);
		}
		else{
			printf("write failed\n");
			//readToNewline(arduino_fd, buffer, 8);
			readFully(arduino_fd, buffer, 6);
			printf("%s\n", buffer);

		}
		usleep(100000);

	}
	return(0);
}
