/*
 * ASIOSerialPort.h
 *
 *  Created on: Nov 8, 2012
 *      Author: Matthew Barulic
 */

#ifndef ASIOSERIALPORT_H_
#define ASIOSERIALPORT_H_

#include <boost/asio.hpp>
#include <string>

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
	void write(std::string s);

	/**
	 * Reads bytes from the serial port until \n or \r is found.
	 * Returns a string containing the bytes read excluding the newline.
	 */
	std::string readln();

	~ASIOSerialPort();
private:
	boost::asio::io_service ioservice;
	boost::asio::serial_port port;

};

#endif /* ASIOSERIALPORT_H_ */
