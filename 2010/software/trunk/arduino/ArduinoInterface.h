#ifndef ARDUINO_INTERFACE_H
#define ARDUINO_INTERFACE_H

#include <cstdlib>
#include <termios.h>  /* POSIX terminal control definitions */
#include <list>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include "DataPacket.hpp"

/** This class defines a standard interface to the arduino microcontroller boards
 *		communicating across a serial interface (rs232, USB, etc.)
 *		To use it, create a driver class that inherts this class which sends and
 *		recieves numbered varables.  These state variables should be the same as
 *		on the Arduino, which can be programmed using the arduino library.
 */

typedef unsigned char byte;

class ArduinoInterface
{
public:
	//bool sendCommand(char cmd, void * data_tx, int size_tx, void * data_rx, int size_rx);
	bool sendCommand(byte cmd, void* data_tx, int size_tx);
	bool recvCommand(byte& out_cmd, byte*& out_data_rx);// I new, you delete

	/* Retrieve all of the state variables in the arduino */
	//bool getStatus(DataPacket& out_status); // TODO: can this be a reference?

	/* Set the value of one the state variables in the arduino */
	bool setVar(int var, void *value, int size);

	/* Destructor */
	virtual ~ArduinoInterface(void);

	//protected:
	/* Constructor - declaired protected to prevent direct instantiation */
	ArduinoInterface(void);
	bool initLink(byte arduinoID);

private:
	/* Private serial interface */
	int arduinoFD;
	bool writeFully(int fd, const void* buf, size_t numBytes);
	bool readFully(int fd, void* buf, size_t numBytes);
	int serialportInit(const char* serialport, speed_t baud);
	bool serialFlush(int fd);

	std::list<DataPacket> tx_packet_list;
	unsigned int rx_num;
	unsigned int tx_num;
	void savePacket(DataPacket pk);
	DataPacket getSavedPacket(int packnum);
public:
	bool arduinoResendPacket(int pknum, DataPacket& pk_out);
	struct timeval getTime();
	bool setArduinoTime();

	bool sendPacket(const DataPacket& pkout);
	bool getPacket(DataPacket& out_pk_rx);
	bool read_TimeOut(int fd, void * buf, size_t numBytes);

	int serialportInit_BOOST(const char* serialport, int baud);
	bool readFully_BOOST(void* buf, size_t numBytes);
	bool writeFully_BOOST(const void* buf, size_t numBytes);
	boost::asio::io_service my_io_service;
	boost::asio::serial_port* asioserialport;
};

#endif /* ARDUINO_INTERFACE_H */

