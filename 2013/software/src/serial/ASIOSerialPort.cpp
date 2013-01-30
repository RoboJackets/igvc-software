/*
 * ASIOSerialPort.cpp
 *
 *  Created on: Nov 8, 2012
 *      Author: Matthew Barulic
 */

#include "ASIOSerialPort.h"
#include <iostream>

ASIOSerialPort::ASIOSerialPort(std::string port_name, size_t baud):
	port(ioservice, port_name),
	eventThread(boost::bind(&ASIOSerialPort::eventThreadRun, this))
{
	if( !port.is_open() ) {
		std::cerr << "Failed to open serial port: " << port_name << std::endl;
		exit(1);
	}

	try {
		port.set_option(boost::asio::serial_port_base::baud_rate(baud));
		port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	} catch(...) {
		std::cerr << "Failed to set all options on port: " << port_name << std::endl;
		exit(1);
	}



	_line = "";
}

void ASIOSerialPort::startEvents()
{
    //TODO ASIOSerialPort::startEvents()
    _eventsEnabled = true;
}

void ASIOSerialPort::eventThreadRun()
{
    while(isConnected())
    {
        if(_eventsEnabled)
        {
            char in = read();
            onNewByte(in);
            if(in == '\n' || in == '\r')
            {
                onNewLine(_line);
                _line = "";
            } else {
                _line += in;
            }

        }
    }
}

void ASIOSerialPort::close() {
	port.close();
}

bool ASIOSerialPort::isConnected() {
	return port.is_open();
}

void ASIOSerialPort::write(std::string s) {
	boost::asio::write(port, boost::asio::buffer(s.c_str(),s.size()));
}

void ASIOSerialPort::write(char *msg, int length)
{
    boost::asio::write(port, boost::asio::buffer(msg, length));
}

std::string ASIOSerialPort::readln() {
    _eventsEnabled = false;
	char c;
	std::string line;
	int nTimes = 0;


	while(true) {
		try {
            boost::asio::read(port, boost::asio::buffer(&c,1));
		} catch(boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >& err) {
			std::cerr << "Error reading stream. Device may have been unplugged." << std::endl;
			std::cout<<"Error reading stream. Device may have been unplugged."<<std::endl;
			std::cout<<nTimes<<std::endl;
			return line;
		}
		switch(c) {
		case '\r':
            _eventsEnabled = true;
			return line;
			break;
		case '\n':
			return line;
			break;
		default:
			line += c;
			nTimes++;
			std::cout<<nTimes<<std::endl;
			break;
		}
	}

    _eventsEnabled = true;
	return "";
}

char ASIOSerialPort::read()
{
    char in;
    try {
        boost::asio::read(port, boost::asio::buffer(&in, 1));
    } catch (boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >& err) {
        std::cerr << "Error reading stream. Device may have been unplugged." << std::endl;
        return 0;
    }
    return in;
}

char* ASIOSerialPort::read(int numBytes)
{
    char* bytes;
    for(int i = 0; i < numBytes; i++)
    {
        bytes[i] = read();
    }
    return bytes;
}

ASIOSerialPort::~ASIOSerialPort() {
}
