/*
 * ASIOSerialPort.h
 *
 *  Created on: Nov 8, 2012
 *      Author: Matthew Barulic
 */

#ifndef ASIOSERIALPORT_H_
#define ASIOSERIALPORT_H_

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <string>
#include <common/events/Event.hpp>

using namespace std;

/**
 * This is a helper class to simplify the interface for interacting with serial ports.
 *
 */
class ASIOSerialPort {
public:
	/**
	 * The constructor takes in the path to the port (eg. "/dev/ttyUSB0") and a baud rate for the connection and opens the connection.
	 */
	ASIOSerialPort(std::string port_name, size_t baud);

    /**
     * Starts the thread for triggering events.
     * Disables synchronous read methods.
     */
     void startEvents();

     /**
      * Stops the thread for triggering events.
      * Reenables synchronous read methods.
      */
      void stopEvents();

	/**
	 * Closes the serial connection.
	 */
	void close();

	/**
	 * Returns true if the serial port is connected and open
	 */
	bool isConnected();

	/**
	 * Writes the given string to the serial port.
	 */
	void write(std::string msg);

    /**
     * Writes the given array of chars to the serial port.
     */
	void write(char *msg, int length);

    /**
     * Writes the given array of unsigned chars to the serial port.
     */
    void write(unsigned char *msg, int length);

	/**
	 * Reads bytes from the serial port until \n or \r is found.
	 * Returns a string containing the bytes read excluding the newline.
	 */
	std::string readln();

	/**
	 * Reads a single byte from the serial port.
	 * Returns the read byte.
	 */
    char read();

    /**
     * Reads numBytes bytes from the serial port.
     * Returns an array containing the read bytes.
     */
     char* read(int numBytes);

     /**
      * Defines the start and end bytes that will trigger a onNewPacket event.
      * NOTE: You must call startEvents() for the onNewPacket event to fire.
      */
     void definePacket(char startByte, char endByte);

    Event<string> onNewLine;
    Event<char> onNewByte;
    Event<string> onNewPacket;

	~ASIOSerialPort();
private:
	boost::asio::io_service ioservice;
	boost::asio::serial_port port;

	boost::thread eventThread;
	boost::mutex portLocker;
	void eventThreadRun();

	bool _eventsEnabled;
	int _eventRequests;

	char _packetStartByte;
	char _packetEndByte;
	bool _packetHasBeenDefined;
	bool _hasEncounteredStartByte;
	string _packet;

    string _line;
};

#endif /* ASIOSERIALPORT_H_ */
