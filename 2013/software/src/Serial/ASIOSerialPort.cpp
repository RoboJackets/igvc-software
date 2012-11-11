/*
 * ASIOSerialPort.cpp
 *
 *  Created on: Nov 8, 2012
 *      Author: Matthew Barulic
 */

#include "ASIOSerialPort.h"
#include <iostream>

ASIOSerialPort::ASIOSerialPort(std::string port_name, size_t baud):
	port(ioservice, port_name)
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

std::string ASIOSerialPort::readln() {
	char c;
	std::string line;

	while(true) {
		try {
		boost::asio::read(port, boost::asio::buffer(&c,1));
		} catch(boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> > err) {
			std::cerr << "Error reading stream. Device may have been unplugged." << std::endl;
			return line;
		}
		switch(c) {
		case '\r':
			return line;
			break;
		case '\n':
			return line;
			break;
		default:
			line += c;
			break;
		}
	}

	return "";
}

ASIOSerialPort::~ASIOSerialPort() {
}
