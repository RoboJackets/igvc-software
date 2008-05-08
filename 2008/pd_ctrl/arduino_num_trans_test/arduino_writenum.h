#ifndef ARDUINO_WRITENUM_H
#define ARDUINO_WRITENUM_H

#include "arduino_comm.h"

typedef unsigned char byte;

bool writeFloat(int fd, float flt){
	byte* outptr = (byte*) &flt;
	return( writeFully(fd, outptr, 4) );
}

bool writeSint16(int fd, short int sint){
	byte* outptr = (byte*) &sint;
	return( writeFully(fd, outptr, 2) );
}

bool writeUint16(int fd, unsigned short int uint){
	byte* outptr = (byte*) &uint;
	return( writeFully(fd, outptr, 2) );
}

bool writeFloat(int fd, float flt, char * cmd){
	writeFully(fd, cmd, 1);

	byte* outptr = (byte*) &flt;
	return( writeFully(fd, outptr, 4) );
}

bool writeSint16(int fd, short int sint, char * cmd){
	writeFully(fd, cmd, 1);

	byte* outptr = (byte*) &sint;
	return( writeFully(fd, outptr, 2) );
}

bool writeUint16(int fd, unsigned short int uint, char * cmd){
	writeFully(fd, cmd, 1);

	byte* outptr = (byte*) &uint;
	return( writeFully(fd, outptr, 2) );
}

#endif //ARDUINO_WRITENUM_H
