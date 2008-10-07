#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h>
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <list>
#include <string.h>

#include "ArduinoInterface.h"
#include "DataPacket.h"

// TODO:	add a timeouts to serial functions
//			add checks for disconnected arduino
//			add error reporting

#define WAIT_TIME 20 //Used for setting VTIME, each tick is 0.1 s
#define BAUD B9600  //Serial baudrate
#define SERIAL_PORT "/dev/ttyUSB0"
#define MODULE_ID 'e'


/**
 * Opens a connection to an arduino.
 * 
 */
ArduinoInterface::ArduinoInterface(void) {
	/* See if anything is connected on ports USB0 through USB9 */
	char serialAddress[] = SERIAL_PORT;
	for (int i=0;i<10;i++) {
		printf("Trying port #%d:  ", (int) i);

		serialAddress[11]=i+'0';
		arduinoFD = serialportInit(serialAddress, BAUD);
		if (arduinoFD == -1) {
			printf("no module found.\n");
			//retry//no module found
		}
		else {
			/* Check which arduino is connected to the port */
			byte reply;
			writeFully(arduinoFD, (void *)"i", 1);
			readFully(arduinoFD, &reply, sizeof(reply));
			printf("found module %c, ", reply);
			if (reply=='e') { //might want to use more than one byte for identifcation
				printf("correct module.\n");
				break;
			} else {
				printf("wrong module.\n");
			}
		}
	}
	if (arduinoFD == -1) {
		printf("Error: MotorEncoders: could not find arduino\n");
		exit(1);
	}
}

/**
 * Closes the connection to the arduino.
 * 
 */
ArduinoInterface::~ArduinoInterface(void) {
	if (arduinoFD == -1) {
		return;
	}
	close(arduinoFD);
}

/**
 * Reads the status of the motor controller.
 * 
 * @param status	OUT: the motor controller's status.
 * @param size		the size of status in bytes.
 * @return			<tt>true</tt> if an error occurs;
 * 					<tt>false</tt> if successful.
 *					More detailed information can be obtained by querying
 *					<tt>errno</tt> and <tt>strerror(errno)</tt>.
 */
bool ArduinoInterface::getStatus(void *status, int size) {
	if (arduinoFD == -1) {
		return false;
	}

	if (writeFully(arduinoFD, (void *)"r", 1)) {
		return true;
	}

	if (readFully(arduinoFD, status, size)) {
		return true;
	}

	return false;
}

/**
 * Set the specified variable in the motor controller to the specified value.
 * 
 * @param var		the variable to change.
 * @param value		the new value for the variable.
 * @param size		size of value in bytes.
 * @return			<tt>true</tt> if an error occurs;
 * 					<tt>false<writeFully(arduinoFD, "r", 1)/tt> if successful.
 *					More detailed information can be obtained by querying
 *					<tt>errno</tt> and <tt>strerror(errno)</tt>.
 */
bool ArduinoInterface::setVar(int var, void *value, int size) {
	if (arduinoFD == -1) {
		return false;
	}

	char buf[2+size]; // TODO: is there a better way to initalize this?
	buf[0] = 'w';
	buf[1] = var;
	for (int i = 0; i < size; i++) {
		buf[2+i] = ((byte *)value)[i];
	}
	if (writeFully(arduinoFD, buf, sizeof(buf))) {
		return true;
	}
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
 */
bool ArduinoInterface::writeFully(int fd, void* buf, size_t numBytes) {
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
 */
bool ArduinoInterface::readFully(int fd, void* buf, size_t numBytes) {
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
 */
int ArduinoInterface::serialportInit(const char* serialport, speed_t baud) {
	struct termios toptions;
	int fd;
	
	//fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
	//		serialport,baud);

	fd = open(serialport, (O_RDWR | O_NOCTTY | O_NDELAY) &~ O_NONBLOCK);
	if (fd == -1)  {
		//perror("serialport_init: Unable to open port ");
		return -1;
	}
	
	if (tcgetattr(fd, &toptions) < 0) {
		//perror("serialport_init: Couldn't get term attributes");
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
		//perror("serialport_init: Couldn't set term attributes");
		return -1;
	}

	return fd;
}

/**
 * Clear any bytes in the incoming serial buffer. TODO: do this for outgoing buffer, too.
 *
 * @param fd		the file to clear.
 */
bool ArduinoInterface::serialFlush(int fd) { // TODO: is there a function for this?
	char dst[1];
	ssize_t numRead = 0;
	
	while (numRead) {
		numRead = read(fd, dst, 1);
	}
	if (numRead < 0) {
		return true;
	}
}


void ArduinoInterface::savePacket(int packnum, size_t len, void * data){

	if(packetlist.size() > 50){
		packetlist.pop_back();
	}


	PCdatapacket stordata;
	stordata.packnum = packnum;
	stordata.len = len;
	stordata.data = new byte[len];
	memcpy(stordata.data, data, len);

	packetlist.push_front(stordata);
}

void ArduinoInterface::deletePacket(int packnum){
	std::list<PCdatapacket>::iterator it;
	for(it = packetlist.begin(); it != packetlist.end(); it++){
		if(it->packnum == packnum){
			//delete[] it.data;
			packetlist.erase(it);
		}
	}
}

PCdatapacket ArduinoInterface::getPacket(int packnum){
	std::list<PCdatapacket>::iterator it;
	for(it = packetlist.begin(); it != packetlist.end(); it++){
		if(it->packnum == packnum){
			PCdatapacket out = *it;			
			return(out);
		}
	}
}



