#pragma once
#include <string>
#include <boost/asio.hpp>

class SerialPort
{
public:
    SerialPort(std::string device, int baud);
    
    ~SerialPort();
    
    bool isOpen();
    
    void write(std::string msg);
    
    char read();
    
    std::string readln();
    
private:
    boost::asio::io_service ioservice;
    boost::asio::serial_port port;

};
