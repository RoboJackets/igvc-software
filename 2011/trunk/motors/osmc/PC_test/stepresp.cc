
#include <iostream>
#include <ctime>
#include "OSMC_driver.hpp"
#include "quadCoderDriver.hpp"
//#include <>

//static const int speedsetdel = 3 * 1e4;
static const int speedsetdel = 2 * 1e4;

static int stepmag = 60;

int main()
{
	OSMC_driver motordriver;
	quadCoderDriver qD;

	motordriver.setMotorPWM(MC_MOTOR_FORWARD, stepmag, MC_MOTOR_FORWARD, stepmag);
	timeval t0;
	gettimeofday(&t0, NULL);
	//for(int i = 0; i < 10; i++)
	for(;;)
	{
		double r = -1, l = -1;
		if(!qD.getEncoderVel(r,l))
		{
			timeval now;
			gettimeofday(&now, NULL);
			double t = double(now.tv_sec - t0.tv_sec) + ( double(1e-6)*double(now.tv_usec) - double(1e-6)*double(t0.tv_usec) );
			std::cout << "t: " << t << " motor" << stepmag << " r: " << r << " l: " << l << std::endl;
		}	
		usleep(1e6);
	}

}
