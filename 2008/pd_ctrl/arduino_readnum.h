#ifndef ARDUINO_READNUM_H
#define ARDUINO_READNUM_H

#include "arduino_comm.h"

typedef unsigned char byte;

float readFloat(int fd){

	char buffin[4];

	readFully(fd, buffin, 4);

	float out;
	byte * outptr = (byte*) &out;
	outptr[0] = (byte) buffin[0];
	outptr[1] = (byte) buffin[1];
	outptr[2] = (byte) buffin[2];
	outptr[3] = (byte) buffin[3];

	return(out);
}

short int readSint16(int fd){
	char buffin[2];
	readFully(fd, buffin, 2);

	short int out;
	byte * outptr = (byte*) &out;
	outptr[0] = (byte) buffin[0];
	outptr[1] = (byte) buffin[1];

	return(out);

}

unsigned short int readUint16(int fd){
	char buffin[2];
	readFully(fd, buffin, 2);

	unsigned short int out;
	byte * outptr = (byte*) &out;
	outptr[0] = (byte) buffin[0];
	outptr[1] = (byte) buffin[1];
	return(out);
}

float readFloat(int fd, char * cmd){
	char buffin[4];

	writeFully(fd, cmd, 1);
	readFully(fd, buffin, 4);

	float out;
	byte * outptr = (byte*) &out;
	outptr[0] = (byte) buffin[0];
	outptr[1] = (byte) buffin[1];
	outptr[2] = (byte) buffin[2];
	outptr[3] = (byte) buffin[3];

	return(out);
}

short int readSint16(int fd, char * cmd){
	char buffin[2];

	writeFully(fd, cmd, 1);
	readFully(fd, buffin, 2);

	short int out;
	byte * outptr = (byte*) &out;
	outptr[0] = (byte) buffin[0];
	outptr[1] = (byte) buffin[1];

	return(out);

}

unsigned short int readUint16(int fd, char * cmd){
	char buffin[2];

	writeFully(fd, cmd, 1);
	readFully(fd, buffin, 2);

	unsigned short int out;
	byte * outptr = (byte*) &out;
	outptr[0] = (byte) buffin[0];
	outptr[1] = (byte) buffin[1];
	return(out);
}

#endif //ARDUINO_READNUM_H
