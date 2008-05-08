#include <unistd.h>   /* UNIX standard function definitions */
#include <cstdio>    /* Standard input/output definitions */
#include <cassert>

#include "arduino_comm.h"
#include "arduino_readnum.h"
#include "arduino_writenum.h"

#define SERIALPORT "/dev/ttyUSB0"
#define BAUD B9600

int main(void){

//	char buffer[10] = {0};
	char f[2] = "f";
	char b[2] = "b";
	char u[2] = "u";

	char x[2] = "x";

	int arduino_fd = serialport_init(SERIALPORT, BAUD);
	assert(arduino_fd != -1);

	printf("a short int is %u bytes long\n",sizeof(short int));

	float xfer_f;
	xfer_f = readFloat(arduino_fd, f);
	printf("rec float: %f\n", xfer_f);
	
	short int xfer_si;
	xfer_si = readSint16(arduino_fd, b);
	printf("rec sint16: %d\n", xfer_si);

	unsigned short int xfer_usi;
	xfer_usi = readUint16(arduino_fd, u);
	printf("rec uint16: %d\n", xfer_usi);

	unsigned short int uint16 = 5;
	unsigned short int sint16 = -5;
	float flt = -6.78;

//	writeFloat(arduino_fd, flt);
	writeSint16(arduino_fd, sint16, x);
//	writeUint16(arduino_fd, uint16);

}
