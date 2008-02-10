
#ifndef _CANDII_SERIAL_H_
#define _CANDII_SERIAL_H_

#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h> 
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <stdbool.h>

bool readFully(int fd, void* buf, size_t numBytes);
bool writeFully(int fd, void* buf, size_t numBytes);
int serialport_init(const char* serialport, int baud);


#endif /* _CANDII_SERIAL_H_ */
