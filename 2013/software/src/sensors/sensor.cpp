#include "sensor.h"

sensor::sensor() : port(io_service) {
	// TODO Auto-generated constructor stub

}

bool sensor::open(const std::string& device, size_t baud) {
	try
	{
		port.open(device);
		std::cout << "Everything Worked, Brah" << std::endl;
		return true;
	}
	catch(...)
	{
		std::cerr << "Failed to open serial port" << std::endl;
		//stacktrace();
		return false;
	}
	return false;
}

void sensor::close() {

		port.close();
		//m_connected = false;
}


sensor::~sensor() {
	port.close();
	// TODO Auto-generated destructor stub
}

