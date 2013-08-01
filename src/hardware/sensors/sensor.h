
#ifndef SENSOR_H_
#define SENSOR_H_

#include <boost/asio.hpp>
#include <boost/thread.hpp>

class sensor {
public:
	sensor();
	virtual ~sensor();

private:
	boost::thread iothread;

};

#endif /* SENSOR_H_ */
