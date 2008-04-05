#include <stdio.h>    /* Standard input/output definitions */

#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <stdbool.h>

#include "motors.h"

// ### PRIVATE FUNCTION PROTOTYPES ###

bool readFully(int fd, void* buf, size_t numBytes);
bool writeFully(int fd, void* buf, size_t numBytes);
int serialport_init(const char* serialport, speed_t baud);

// ### PUBLIC FUNCTIONS: MOTORS API ###

#define WAIT_TIME 20 //Used for setting VTIME, each tick is 0.1 s
#define BAUD B9600  //Serial baudrate
#define DEFAULT_SERIAL "/dev/ttyUSB0"
char serialAddress[]="/dev/ttyUSB0";
static int arduino_fd;

/**
 * Opens a connection to the motors.
 * 
 * @return			<tt>true</tt> if the connection to the motors could not be opened;
 * 					<tt>false</tt> if successful.
 * @author David Foster & Kurt Tomlinson
 */
bool motors_open(void) {
	int i;
	for (i=0;i<10;i++)
	{	
		printf("trying port #%d\n", (int) i);

		serialAddress[11]=i+'0';
		arduino_fd = serialport_init(serialAddress, BAUD);
		if (arduino_fd == -1)
		{
			//printf("no module\n");
			//retry//no module found
		}
		else
		{
			unsigned char reply;
			writeFully(arduino_fd, "i", 1);
			readFully(arduino_fd, &reply, sizeof(reply));
			if (reply=='m')
			{
				//printf("DONE!!\n");
				break;
			}
			else
			{
				//printf("wrong module\n");
				//retry//wrong module found
			}
		}
	}
	return (arduino_fd == -1);
}

/**
 * Closes the connection to the motors.
 * 
 * @author David Foster
 */
void motors_close(void) {
	if (arduino_fd == -1) return;
	close(arduino_fd);
}

/**
 * Reads the status of the motor controller.
 * 
 * @param status	OUT: the motor controller's status.
 * @return			<tt>true</tt> if an error occurs;
 * 					<tt>false</tt> if successful.
 *					More detailed information can be obtained by querying
 *					<tt>errno</tt> and <tt>strerror(errno)</tt>.
 * @author David Foster
 */
bool motors_getStatus(motor_reply_t *status) {
	if (arduino_fd == -1) return false;
	
	if (writeFully(arduino_fd, "r", 1))
		return true;
	
	if (readFully(arduino_fd, status, sizeof(*status)))
		return true;
	
	return false;
}

/**
 * Set the specified variable in the motor controller to the specified value.
 * 
 * @param var		the variable to change.
 * @param value		the new value for the variable.
 * @return			<tt>true</tt> if an error occurs;
 * 					<tt>false<writeFully(arduino_fd, "r", 1)/tt> if successful.
 *					More detailed information can be obtained by querying
 *					<tt>errno</tt> and <tt>strerror(errno)</tt>.
 * @author David Foster
 */
bool motors_setVar(motor_var_t var, char value) {
	if (arduino_fd == -1) return false;
	
	char buf[3] = {'w',var,value};
	if (writeFully(arduino_fd, buf, sizeof(buf)))
		return true;
	return false;
}

// ### PRIVATE FUNCTIONS: SERIAL API ###

/**
 * Write exactly <tt>numBytes</tt> bytes to file <tt>fd</tt>
 * from buffer <tt>buf</tt>.
 * 
 * @param fd		the file to write to.
 * @param buf		the buffer to read from.
 * @param numBytes	the number of bytes to write.
 * @return			<tt>true</tt> if an error occurs;
 * 					<tt>false</tt> if successful.
 * @author David Foster
 */
bool writeFully(int fd, void* buf, size_t numBytes) {
	char* src = (char*) buf;
	size_t numLeft = numBytes;
	ssize_t numWritten;
	
	while (numLeft > 0) {
		numWritten = write(fd, src, numLeft);
		if (numWritten < 0) {
			return true;
		}
		
		numLeft -= numWritten;
		src += numWritten;
	}
	return false;
}

/**
 * Read exactly <tt>numBytes</tt> bytes from file <tt>fd</tt>
 * into buffer <tt>buf</tt>.
 * 
 * @param fd		the file to read from.
 * @param buf		the buffer to read into.
 * @param numBytes	the number of bytes to read.
 * @return			<tt>true</tt> if an error occurs;
 * 					<tt>false</tt> if successful.
 * @author David Foster
 */
bool readFully(int fd, void* buf, size_t numBytes) {
	char* dst = (char*) buf;
	size_t numLeft = numBytes;
	ssize_t numRead;
	
	while (numLeft > 0) {
		numRead = read(fd, dst, numLeft);
		if (numRead < 0) {
			return true;
		}
		
		numLeft -= numRead;
		dst += numRead;
	}
	return false;
}

/**
 * Open a serial port at device <tt>serialport</tt> and initialize
 * it for raw mode with a baud rate of <tt>baud</tt>. 
 * All other parameters are set by the function.   
 * 
 * @param serialport	the serial port to open and initialize.
 * @param baud			the baud rate at which to run the serial port.
 * @return 				<tt>fd</tt> the file descriptor for the open serial port;
 *                       -1 if an error occurs.			
 * 					
 * @author Logan Snow
 */
int serialport_init(const char* serialport, speed_t baud) {
	struct termios toptions;
	int fd;
	
	//fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
	//		serialport,baud);

	fd = open(serialport, (O_RDWR | O_NOCTTY | O_NDELAY) &~ O_NONBLOCK);
	if (fd == -1)  {
		perror("serialport_init: Unable to open port ");
		return -1;
	}
	
	if (tcgetattr(fd, &toptions) < 0) {
		perror("serialport_init: Couldn't get term attributes");
		return -1;
	}
	
    cfsetispeed(&toptions, baud);
	cfsetospeed(&toptions, baud);

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;

	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw

	/* see: http://unixwiz.net/techtips/termios-vmin-vtime.html */
	// Set read-timeout for reading each byte
	toptions.c_cc[VTIME] = WAIT_TIME;
	
	if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
		perror("serialport_init: Couldn't set term attributes");
		return -1;
	}

	return fd;
}
