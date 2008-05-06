#include <unistd.h>   /* UNIX standard function definitions */
#include <cstdio>    /* Standard input/output definitions */
#include <errno.h>    /* Error number definitions */
#include <assert.h>
#include <cmath>

#include "arduino_comm.h"
#include "arduino_readnum.h"

#define SERIALPORT "/dev/ttyUSB0"
#define BAUD B9600

//#define DT 100

#define COUNTER_SCALER (64)
#define F_CPU (16000000)
#define COUNTER_RATE (F_CPU/COUNTER_SCALER)
#define RAD_ENCODERTICK ( (2*M_PI)/512 )

int main(void){
	char buffer[6] = {0};
	char v[2] = "v";
	char t[2] = "t";
	char p[2] = "p";
	int arduino_fd = serialport_init(SERIALPORT, BAUD);
	
	unsigned short int dt;
	short int dp;
	float velocity;

	writeFully(arduino_fd, v, 1);
	dt = readUint16(arduino_fd, t, buffer);
	dp = readSint16(arduino_fd, p, buffer);

	velocity = ( (float)dp) / ( (float)dt) * RAD_ENCODERTICK * COUNTER_RATE;
	printf("dp: %d\tdt: %d\tvel: %d\n", dp, dt, velocity);

	return(0);
}

