#ifndef ARDUINO_INTERFACE_H
#define ARDUINO_INTERFACE_H

#include <stdlib.h>
#include <termios.h>  /* POSIX terminal control definitions */
#include <list>
#include "DataPacket.h"

/** This class defines a standard interface to the arduino microcontroller boards
 *		communicating across a serial interface (rs232, USB, etc.)
 *		To use it, create a driver class that inherts this class which sends and
 *		recieves numbered varables.  These state variables should be the same as
 *		on the Arduino, which can be programmed using the arduino library.
 */

typedef unsigned char byte;

class ArduinoInterface {
	public:
		/* Retrieve all of the state variables in the arduino */
		bool getStatus(void *status, int size); // TODO: can this be a reference?

		/* Set the value of one the state variables in the arduino */
		bool setVar(int var, void *value, int size);

		/* Destructor */
		virtual ~ArduinoInterface(void);

	protected:
		/* Constructor - declaired protected to prevent direct instantiation */
		ArduinoInterface(void);

	private:
		/* Private serial interface */
		int arduinoFD;
		bool writeFully(int fd, void* buf, size_t numBytes);
		bool readFully(int fd, void* buf, size_t numBytes);
		int serialportInit(const char* serialport, speed_t baud);
		bool serialFlush(int fd);

		void savePacket(int packnum, void * data, size_t len);
		void deletePacket(int packnum);
		PCdatapacket getPacket(int packnum);

		std::list<PCdatapacket> packetlist;
		int tx_num;
		int rx_num;
};

#endif /* ARDUINO_INTERFACE_H */

