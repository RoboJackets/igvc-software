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

#define DT 100

int main(void){
	char buffer[6] = {0};
	char char_buff;
	char a = 'a';
	int fd = serialport_init(SERIALPORT, BAUD);
	
	int p1,p2;
	struct timeval t1,t2;
	struct timezone *tz;
	float velocity;
	
	while(1){
		//writeFully(fd, &a, 1);
		//usleep(200);		
				
		//gettimeofday(&t1, &tz);
		
		readFully(fd, &buffer, 5);
		if(buffer[0] = "p"){
			p1 = atoi(buffer);
		}
		else{
			t1 = atoi(buffer);
		}

		readFully(fd, &buffer, 5);
		if(buffer[0] = "p"){
			p2 = atoi(buffer);
		}
		else{
			t2 = atoi(buffer);
		}
		

		//gettimeofday(&t2, &tz);
		
		//velocity = (v2 - v1) / (t2.tv_usec - t1.tv_usec) * 1000;
		velocity = (p2 - p1) * ((2* M_PI)/512) / DT;
		printf("%d\n", velocity);
		//printf("time %d\n",(t2.tv_usec - t1.tv_usec));
		}

		//printf("%s\n",buffer);

	return(0);
}

