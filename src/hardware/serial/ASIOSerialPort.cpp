/*
 * ASIOSerialPort.cpp
 *
 *  Created on: Nov 8, 2012
 *      Author: Matthew Barulic
 */

#include "ASIOSerialPort.h"
#include <iostream>
#include <common/logger/logger.h>

ASIOSerialPort::ASIOSerialPort(std::string port_name, size_t baud)
    : port(ioservice)
{
    try
    {
        port.open(port_name);
    } catch(...){}

	if( !port.is_open() ) {
        std::stringstream msg;
        msg << "Failed to open serial port: " << port_name;
        Logger::Log(LogLevel::Error, msg.str());
        //exit(1);
	}

    if(isConnected())
    {
        try {
            port.set_option(boost::asio::serial_port_base::baud_rate(baud));
            port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

        } catch(...) {
            std::stringstream msg;
            msg << "Failed to set all options on port: " << port_name;
            Logger::Log(LogLevel::Error, msg.str());
            //exit(1);
        }
    }


    _packetHasBeenDefined = false;
    _hasEncounteredStartByte = false;
	_line = "";
	_eventRequests = 0;
}

void ASIOSerialPort::startEvents() {
    _eventRequests++;
    if(!_eventsEnabled)
    {
        _eventsEnabled = true;
        eventThread = boost::thread(boost::bind(&ASIOSerialPort::eventThreadRun, this));
    }
}

void ASIOSerialPort::stopEvents() {
    _eventRequests--;
    if(_eventRequests == 0)
    {
        _eventsEnabled = false;
        eventThread.join();
    }

}

void ASIOSerialPort::eventThreadRun() {
    while(isConnected() && _eventsEnabled)
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

void ASIOSerialPort::close() {

    if(_eventsEnabled)
    {
        _eventsEnabled = false;
        eventThread.join();
    }
    if(isConnected())
    {
        port.close();
    }

}

bool ASIOSerialPort::isConnected() {
	return port.is_open();
}

void ASIOSerialPort::write(std::string s) {

    if(isConnected()) boost::asio::write(port, boost::asio::buffer(s.c_str(),s.length()));

}

void ASIOSerialPort::write(char *msg, int length) {

    if(isConnected()) boost::asio::write(port, boost::asio::buffer(msg, length));

}

void ASIOSerialPort::write(unsigned char *msg, int length) {

    if(isConnected()) boost::asio::write(port, boost::asio::buffer(msg, length));

}

std::string ASIOSerialPort::readln() {
    if(!isConnected()) return "";

	char c;
	std::string line;


    while(true) {
        try {

            boost::asio::read(port, boost::asio::buffer(&c,1));

		} catch(boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >& err) {
            Logger::Log(LogLevel::Error, "Error reading serial port. Device may have been uplugged.");
            Logger::Log(LogLevel::Error, err.what());
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
			break;
		}
		if(_packetHasBeenDefined)
		{
		    if(c==_packetStartByte)
		    {
		        _packet = "";
		        _hasEncounteredStartByte = true;
		    }
		    if(_hasEncounteredStartByte)
		    {
		        _packet += c;
		    }
		    if(c==_packetEndByte)
		    {
                onNewPacket(_packet);
                _hasEncounteredStartByte = false;
		    }
		}
	}
	return "";
}

char ASIOSerialPort::read() {
    if(!isConnected()) return -1;


    char in;
    try {

        boost::asio::read(port, boost::asio::buffer(&in, 1));

    } catch (boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >& err) {
        Logger::Log(LogLevel::Error, "Error reading serial port. Device may have been uplugged.");
        Logger::Log(LogLevel::Error, err.what());
        return 0;
    }
    return in;
}

char* ASIOSerialPort::read(int numBytes) {
    if(!isConnected()) return (char*)"";
    char* bytes = new char[numBytes];
    for(int i = 0; i < numBytes; i++)
    {

        bytes[i] = read();

    }
    return bytes;
}

void ASIOSerialPort::definePacket(char startByte, char endByte)
{
    _packetStartByte = startByte;
    _packetEndByte = endByte;
    _packetHasBeenDefined = true;
}

ASIOSerialPort::~ASIOSerialPort() {
    if(_eventsEnabled)
    {
        _eventsEnabled = false;
        eventThread.join();
    }
}
