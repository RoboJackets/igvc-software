#include "gyro.hpp"

#include <iostream>

int main()
{
	gyro gyroA;

	gyroA.open("/dev/ttyUSB0", 19200);
	gyroA.start();

	while(true)
	{
		gyroState state;
		if(gyroA.get_last_state(state))
		{
			printf("<r,p,y>: <%f,%f%f>\tball: %f\tyawrate: %f\n", state.rpy[0], state.rpy[1], state.rpy[2], state.balloffset, state.yawrate);
		}
		else
		{
			std::cout << "Error getting gps state" << std::endl;
		}
		usleep(1e6);
	}
}
