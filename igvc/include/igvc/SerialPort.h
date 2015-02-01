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

    void write(char *buffer, int length);

    void write(unsigned char *buffer, int length);
    
    char read();

    char* read(int numBytes);
    
    std::string readln();

    std::string devicePath();
    
private:
    boost::asio::io_service ioservice;
    boost::asio::serial_port port;
    std::string path;

};
