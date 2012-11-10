
#ifndef SENSOR_H_
#define SENSOR_H_

#include <boost/asio.hpp>
#include <boost/thread.hpp>

class sensor {
public:
	sensor();
	bool sensor::open(const std::string& device, size_t baud);
	void close();
	virtual ~sensor();

private:
	boost::thread iothread;
	boost::asio::io_service io_service;
	boost::asio::serial_port port;

};

#endif /* SENSOR_H_ */
