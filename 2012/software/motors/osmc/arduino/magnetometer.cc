#include "magnetometer.hpp"

magnetometer_pk_t getHeading()
{
	magnetometer_pk_t heading;
	heading.angle = 1347;
	return heading;
	//return a random number for now, but eventually this will actually get the information from the magnetometer and return the angle value.
}
