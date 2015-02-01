/*! \file SerialPort.h
 * \date Created on: Feb 1, 2015
 * \author Matthew Barulic
 */

#pragma once
#include <string>
#include <boost/asio.hpp>

/*!
 * \brief Helper class to simplify interfacing with serial port hardware.
 * \headerfile SerialPort.h <igvc/SerialPort.h>
 */
class SerialPort
{
public:
    /*! \brief The constructor takes in the path to the port (eg. "/dev/ttyUSB0") and a baud rate for the connection and opens the connection. */
    SerialPort(std::string device, int baud);
    
    ~SerialPort();
    
    /*! \brief Returns true if the serial port is connected and open */
    bool isOpen();
    
    /*! \brief Writes the given string to the serial port. */
    void write(std::string msg);

    /*! \brief Writes the given array of chars to the serial port. */
    void write(char *buffer, int length);

    /*! \brief Writes the given array of unsigned chars to the serial port. */
    void write(unsigned char *buffer, int length);
    
    /*! \brief Reads a single byte from the serial port.
     * \return The byte read.
     */
    char read();

    /*! \brief Reads numBytes bytes from the serial port.
     * \return An array containing the read bytes.
     */
    char* read(int numBytes);
    
    /*! \brief Reads bytes from the serial port until \n or \r is found.
     * \return String containing the bytes read excluding the newline.
     */
    std::string readln();

    /*! \brief Returns the path to the device this port is connected to.
     * \return String containing path to device.
     */
    std::string devicePath();
    
private:
    boost::asio::io_service ioservice;
    boost::asio::serial_port port;
    std::string path;

};
