/**
* An abstraction of boost::ip::tcp to ease communication with
* ethernet devices
*
* date created: December 8th, 2018
* author: Alejandro Escontrela
*/

#ifndef ETHERNETSOCKET_H
#define ETHERNETSOCKET_H

#include <boost/asio.hpp>
#include <string>

/**
 * The <code>EthernetSocket</code> class provides a wrapper for the
 * boost::ip::tcp library that is meant to ease communication with
 * ethernet devices.
 */
class EthernetSocket
{
public:
    bool connected;

    /**
    Constructor opens the socket using the tcp v4 protocol and binds it to an
    endpoint. This endpoint is constructed using the specified ip address and
    port, providing the socket with a unique address.

    @param[in] ip_addr ip address to communicate with
    @param[in] port port to communicate through
    @throw std::runtime_error is socket couldn't open or connect
    */
    EthernetSocket(std::string ip_addr, int port);

    ~EthernetSocket();

    /**
    Transmit a std::string to the endpoint

    @param[in] msg the string to transmit
    */
    void write(std::string msg);
    /**
    Transmit a (signed) char array of specified length to the endpoint.
    @param[in] buffer char array to send
    @param[in] length length of char array
    */
    void write(char* buffer, int length);
    /**
    Transmit a (unsigned) char array of specified length to the endpoint.
    @param[in] buffer char array to send
    @param[in] length length of char array
    */
    void write(unsigned char* buffer, int length);
    /**
    Read chars from the buffer until a newline character is encountered.

    @return a string of the characters read
    */
    std::string readln();
    /**
    Read a specified number of bytes from the socket. Return a char array
    containing the read bytes.

    @param[in] numBytes number of bytes to read from the buffer
    @return a char array containing the read bytes
    */
    char* read(int numBytes);
    /**
    Read one byte from the socket. Return a char containing the read byte.

    @return the read byte
    */
    char read();

    /**
    Getter for IP address

    @return ip address as string
    */
    std::string getIPAddress();

    /**
    Getter for port number

    @return port as int
    */
    int getPort();
    /**
    Determine whether the socket is open

    @return boolean denoting whether or not the socket is open
    */
    bool isOpen();


private:
    boost::asio::io_service ioservice; // provides core io functionality
    boost::asio::ip::address ip_address; // ip address object
    boost::asio::ip::tcp::socket s; // socket used for communication

};

#endif
