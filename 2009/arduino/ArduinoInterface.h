#ifndef ARDUINO_INTERFACE_H
#define ARDUINO_INTERFACE_H

#include <stdlib.h>
#include <termios.h>  /* POSIX terminal control definitions */
#include <list>

#include "DataPacket.hpp"

/** This class defines a standard interface to the arduino microcontroller boards
 *		communicating across a serial interface (rs232, USB, etc.)
 *		To use it, create a driver class that inherts this class which sends and
 *		recieves numbered varables.  These state variables should be the same as
 *		on the Arduino, which can be programmed using the arduino library.
 */

typedef unsigned char byte;

class ArduinoInterface {
	public:
		bool sendCommand(char cmd, void * data_tx, int size_tx, void * data_rx, int size_rx);
		//bool sendCommand(char cmd, void * data_tx, int size_tx, DataPacket * pk_rx, size_t maxlen);

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

		std::list<DataPacket*> tx_packet_list;
		int rx_num;
		int tx_num;
		void savePacket(DataPacket* pk);
		DataPacket* getSavedPacket(int packnum);
		public: void arduinoResendPacket(int pknum, DataPacket * pk_out);
		unsigned int getTime();
		bool setArduinoTime();
};

#endif /* ARDUINO_INTERFACE_H */

