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

class EthernetSocket
{
public:
    bool connected;

    EthernetSocket(std::string ip_addr, int port);

    ~EthernetSocket();

    bool isOpen();
    void write(std::string msg);
    void write(char* buffer, int length);
    void write(unsigned char* buffer, int length);

    std::string readln();
    char* read(int numBytes);
    char read();


private:
    boost::asio::io_service ioservice;
    boost::asio::ip::address ip_address;
    boost::asio::ip::tcp::socket s;

};

#endif
