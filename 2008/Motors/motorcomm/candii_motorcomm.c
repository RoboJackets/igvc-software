/*Added command line arguments
when executing the following line at the command line prompt, an 'r' is sent over the serial connection to the arduino and the arduino dumps its data back in response
candii_motorcomm r
in order to write to a variable on the arduino, type the foloowing line into the command prompt. replace #1 with the number of the variable that you wish to write to, and replace #2 with the value you wish to write to that variable.
candii_motorcomm w #1 #2

when #1=0, the variable being written to is "leftMotorSpeed"
when #1=1, the variable being written to is "rightMotorSpeed"
when #1=2, the variable being written to is "softEStop"

leftMotorSpeed and rightMotorSpeed can have values between 0 and 255 inclusive. a value of 128 stops the motors. positive values spin the motors forward. negative values spin the motors in reverse.

setting softEStop to anything other than 0 makes the motors stop moving and sets their speed to 127 (stopped.)
*/

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

#include "candii_motorcomm.h"

#define WAIT_TIME 20 //Used for setting VTIME, each tick is 0.1 s
#define BAUD B9600  //Serial baudrate
#define DEFAULT_SERIAL "/dev/ttyUSB0"

//The structure arduino_motor_reply_t defines and receives data that is read from the arduino-based motor control board.

typedef struct {
	uint8_t HWESTOP; //0 if Hardware Estop is untriggered, 1 if it is
	uint8_t AUTOMAN; //0 if the robot is operating in autonomous mode, 1 if manual control is active
	uint8_t PATHNAV; //0 if the robot is in autonomous path following mode, 1 if the robot is in autonomous waypoint navigation mode
	uint8_t CURRENTLEFT1; //Upper byte of the left motor current sensor's 10 bit reading
	uint8_t CURRENTLEFT2; //Lower byte of the left motor current sensor's 10 bit reading
	uint8_t CURRENTRIGHT1; //Upper byte of the right motor current sensor's 10 bit reading
	uint8_t CURRENTRIGHT2; //Lower byte of the right motor current sensor's 10 bit reading
	uint8_t LOGICBATT1; //Upper byte of the logic battery system's voltage 10 bit reading
	uint8_t LOGICBATT2; //Lower byte of the logic battery system's voltage 10 bit reading
	uint8_t MOTORBATT1; //Upper byte of the motor battery system's voltage 10 bit reading
	uint8_t MOTORBATT2; //Lower byte of the motor battery system's voltage 10 bit reading
} arduino_motor_reply_t;


int main(int argc, char *argv[]) {
	int fd = 0;

	arduino_motor_reply_t reply;
	ssize_t numWritten;
	fd = serialport_init(DEFAULT_SERIAL, BAUD);

	if (argv[1][0]=='r')
	{
		if (writeFully(fd, "r", 1))
		{
			printf("serialport_write: Write error %d: %s\n", (int) errno, (char*) strerror(errno));
		}

		printf("sizeof(reply): %d \n", sizeof(reply));
		if (readFully(fd, &reply, sizeof(reply)))
		{
			printf("serialport_read: Read error %d: %s\n", (int) errno, (char*) strerror(errno));
		}
		printf("HWESTOP %d \n", (int) reply.HWESTOP);
		printf("AUTOMAN %d \n", (int) reply.AUTOMAN);
		printf("PATHNAV %d \n", (int) reply.PATHNAV);
		printf("CURRENTLEFT1 %d \n", (int) reply.CURRENTLEFT1);
		printf("CURRENTLEFT2 %d \n", (int) reply.CURRENTLEFT2);
		printf("CURRENTRIGHT1 %d \n", (int) reply.CURRENTRIGHT1);
		printf("CURRENTRIGHT2 %d \n", (int) reply.CURRENTRIGHT2);
		printf("LOGICBATT1 %d \n", (int) reply.LOGICBATT1);
		printf("LOGICBATT2 %d \n", (int) reply.LOGICBATT2);
		printf("MOTORBATT1 %d \n", (int) reply.MOTORBATT1);
		printf("MOTORBATT2 %d \n", (int) reply.MOTORBATT2);
	}
	if (argv[1][0]=='w')
	{
		char variableName = (char) atoi(argv[2]);
		char variableValue = (char) atoi(argv[3]);
		char buf[3] = {'w',variableName,variableValue};
		if (writeFully(fd, buf, sizeof(buf)))
		{
			printf("serialport_write: Write error %d: %s\n", (int) errno, (char*) strerror(errno));
		}
	}

	close(fd);
}

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




























