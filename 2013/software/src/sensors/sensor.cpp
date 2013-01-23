#include "sensor.h"

sensor::sensor() : port(io_service) {
	// TODO Auto-generated constructor stub

}

sensor::~sensor() {
	port.close();
	// TODO Auto-generated destructor stub
}

