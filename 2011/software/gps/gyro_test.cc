#include "gyro.hpp"

#include <iostream>

int main()
{
	gyro gyroA;

	gyroA.open("/dev/ttyUSB0", 115200);
	gyroA.start();

	while(true)
	{
		gyroState state;
		if(gyroA.get_last_state(state))
		{
			printf("<r,p,y>: <%0.2f,%0.2f,%0.2f>\tball: %0.2f\tyawrate: %0.2f\n", state.rpy[0], state.rpy[1], state.rpy[2], state.balloffset, state.yawrate);
		}
		else
		{
			std::cout << "Error getting gyro state" << std::endl;
		}
		usleep(1e5);
	}
}
