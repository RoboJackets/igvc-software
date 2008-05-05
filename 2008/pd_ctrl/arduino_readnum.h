#ifndef ARDUINO_READNUM_H
#define ARDUINO_READNUM_H

#include "arduino_comm.h"

typedef unsigned char byte;

/*
union byte_to_float{
	float f;
	byte b[4];
};

union byte_to_sint16{
	short int si;
	byte b[2];
};
*/

float readFloat(int fd, char * cmd, char * buffin){
	//byte_to_float b_f;

	writeFully(fd, cmd, 1);
	readFully(fd, buffin, 4);
	/*
	b_f.b[0] = (byte) buffin[0];
	b_f.b[1] = (byte) buffin[1];
	b_f.b[2] = (byte) buffin[2];
	b_f.b[3] = (byte) buffin[3];
	
	return(b_f.f);
	*/

	float out;
	byte * outptr = (byte*) &out;
	outptr[0] = (byte) buffin[0];
	outptr[1] = (byte) buffin[1];
	outptr[2] = (byte) buffin[2];
	outptr[3] = (byte) buffin[3];

	return(out);
}

short int readSint16(int fd, char * cmd, char * buffin){
	//byte_to_sint16 b_si;

	writeFully(fd, cmd, 1);
	readFully(fd, buffin, 2);
/*
	b_si.b[0] = (byte) buffin[0];
	b_si.b[1] = (byte) buffin[1];
	
	return(b_si.si);
*/
	short int out;
	byte * outptr = (byte*) &out;
	outptr[0] = (byte) buffin[0];
	outptr[1] = (byte) buffin[1];

	return(out);

}

unsigned short int readUint16(int fd, char * cmd, char * buffin){
	writeFully(fd, cmd, 1);
	readFully(fd, buffin, 2);

	unsigned short int out;
	byte * outptr = (byte*) &out;
	outptr[0] = (byte) buffin[0];
	outptr[1] = (byte) buffin[1];
	return(out);
}

#endif //ARDUINO_READNUM_H
