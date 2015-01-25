#include <igvc/SerialPort.h>
#include <iostream>

using namespace std;
using namespace boost::asio;

SerialPort::SerialPort(string device, int baud)
    : port(ioservice)
{
    try
    {
        port.open(device);
    } catch(...){}
    
    if( !port.is_open() ) {
        cerr << "Could not open serial port " << device << endl;
        return;
    }
    
    try
    {
        port.set_option(boost::asio::serial_port_base::baud_rate(baud));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    } catch(...) {
        cerr << "Could not set options on serial port " << device << endl;
    }
}

SerialPort::~SerialPort()
{
    port.close();
}

bool SerialPort::isOpen()
{
    return port.is_open();
}

void SerialPort::write(string msg)
{
    if(port.is_open()) boost::asio::write(port, boost::asio::buffer(msg.c_str(),msg.length()));
}

char SerialPort::read()
{
    if(!port.is_open()) return -1;
    
    char in;
    try
    {
        boost::asio::read(port, buffer(&in, 1));
    } catch (boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >& err) {
        cerr << "Error reading serial port." << endl;
        cerr << err.what() << endl;
        return 0;
    }
    return in;
}

string SerialPort::readln()
{
    string line = "";
    while(true)
    {
        char in = read();
        if(in == '\n')
            return line;
        line = line + in;
    }
}
