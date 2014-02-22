/*! \file ASIOSerialPort.h
 * \date Created on: Nov 8, 2012
 * \author Matthew Barulic
 */

#ifndef ASIOSERIALPORT_H_
#define ASIOSERIALPORT_H_

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <string>
#include <QObject>

/*!
 * \brief Helper class to simplify interfacing with serial port hardware.
 * \headerfile ASIOSerialPort.h <hardware/serial/ASIOSerialPort.h>
 */
class ASIOSerialPort : public  QObject {
    Q_OBJECT

public:
    /*! \brief The constructor takes in the path to the port (eg. "/dev/ttyUSB0") and a baud rate for the connection and opens the connection. */
	ASIOSerialPort(std::string port_name, size_t baud);

    /*! \brief Starts the thread for triggering events.
     *
     * Disables synchronous read methods.
     */
     void startEvents();

     /*! \brief Stops the thread for triggering events.
      *
      * Reenables synchronous read methods.
      */
      void stopEvents();

    /*! \brief Closes the serial connection. */
	void close();

    /*! \brief Returns true if the serial port is connected and open */
	bool isConnected();

    /*! \brief Writes the given string to the serial port. */
	void write(std::string msg);

    /*! \brief Writes the given array of chars to the serial port. */
	void write(char *msg, int length);

    /*! \brief Writes the given array of unsigned chars to the serial port. */
    void write(unsigned char *msg, int length);

    /*! \brief Reads bytes from the serial port until \n or \r is found.
     * \return String containing the bytes read excluding the newline.
	 */
	std::string readln();

    /*! \brief Reads a single byte from the serial port.
     * \return The byte read.
	 */
    char read();

    /*! \brief Reads numBytes bytes from the serial port.
     * \return An array containing the read bytes.
     */
     char* read(int numBytes);

     /*!
      * \brief Defines the start and end bytes that will trigger a onNewPacket event.
      * \note You must call startEvents() for the onNewPacket event to fire.
      */
     void definePacket(char startByte, char endByte);

     ~ASIOSerialPort();

signals:
     /*! \brief onNewLine Fires every time a new line character is found in the stream.
      * \note You must call startEvents() for this event to fire.
      */
     void onNewLine(std::string line);
     /*! \brief onNewByte Fires every time a byte is found in the stream.
      * \note You must call startEvents() for this event to fire.
      */
     void onNewByte(char byte);
     /*! \brief onNewPacket Fires every time a user-define packet is found in the stream.
      * \note You must call startEvents() for this event to fire.
      * \note You must define a packet with definePacket() for this event to fire.
      * \see definePacket
      */
     void onNewPacket(std::string packet);

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
    std::string _packet;

    std::string _line;
};

#endif /* ASIOSERIALPORT_H_ */
