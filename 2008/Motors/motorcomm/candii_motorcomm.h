#ifndef CANDII_MOTORCOMM_H
#define CANDII_MOTORCOMM_H

#include <termios.h>  // for speed_t
#include <stdbool.h>  // for bool

bool readFully(int fd, void* buf, size_t numBytes);
bool writeFully(int fd, void* buf, size_t numBytes);
int serialport_init(const char* serialport, speed_t baud);

#endif /* CANDII_MOTORCOMM_H */
