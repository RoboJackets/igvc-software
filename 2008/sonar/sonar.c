//use modes 4,5,6 & up for testing
//mode 4:				manual simultaneous ping
//mode 5:				manual ping sweep (lo->hi)
//mode 6:				manual ping sweep (hi->lo)
//mode 0xE0-0xFE:		manual single ping of sensor #mode

#include <stdio.h>   /* Standard types */
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <stdbool.h>

#include <stdlib.h>
#include <termios.h>  // for speed_t
#include <stdbool.h>  // for bool

// ### PRIVATE FUNCTION PROTOTYPES ###

bool readFully(int fd, void* buf, size_t numBytes);
bool writeFully(int fd, void* buf, size_t numBytes);
int serialport_init(const char* serialport, speed_t baud);

// ### PUBLIC FUNCTIONS: MOTORS API ###

#define WAIT_TIME 20 //Used for setting VTIME, each tick is 0.1 s
#define BAUD B9600  //Serial baudrate
#define DEFAULT_SERIAL "/dev/ttyUSB0"

static int arduino_fd;

int main()
{
	char command[10];
	unsigned char commandVal;
	unsigned char rawReadings[22];
	int readings[11];
	int i;
	printf("0xE0	0xE2	0xE4	0xE6	0xE8	0xEA	0xEC	0xEE	0xF0	0xF2	0xF4\n", commandVal);
	printf("=	=	=	=	=	=	=	=	=	=	=\n", commandVal);
	printf("224	226	228	230	232	234	236	238	240	242	244\n", commandVal);
	
	arduino_fd = serialport_init(DEFAULT_SERIAL, BAUD);
	if (arduino_fd == -1)
	{
		return -1;
	}
	
	printf("mode: ");
	fgets(command,10,stdin);
	commandVal=atoi(command);
	printf("Executing mode %d\n", (int)commandVal);
	
	writeFully(arduino_fd, &commandVal, 1);
	sleep(1);
	readFully(arduino_fd, rawReadings, 22);
	
	for (i=0;i<11;i++)
	{
		readings[i] = (rawReadings[2*i]<<8) | (rawReadings[2*i+1]);
	}
	printf("%d	%d	%d	%d\n\n\n",(int)rawReadings[0],(int)rawReadings[1],(int)rawReadings[2],(int)rawReadings[3]);
	printf("%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d\n\n\n",readings[0],readings[1],readings[2],readings[3],readings[4],readings[5],readings[6],readings[7],readings[8],readings[9],readings[10]);
	close(arduino_fd);
	return 0;
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
