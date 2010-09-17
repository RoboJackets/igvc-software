
#include <iostream>
#include <ctime>
#include "OSMC_4wd_driver.hpp"

//static const int speedsetdel = 3 * 1e4;
static const int speedsetdel = 2 * 1e4;

static int stepmag = 60;

int main()
{
	OSMC_4wd_driver motordriver(OSMC_IF_FOR_BOARD, ENCODER_IF_FOR_BOARD, OSMC_IF_AFT_BOARD, ENCODER_IF_AFT_BOARD);

	motordriver.setMotorPWM(MC_MOTOR_FORWARD, stepmag, MC_MOTOR_FORWARD, stepmag,MC_MOTOR_FORWARD, stepmag, MC_MOTOR_FORWARD, stepmag);
	timeval t0;
	gettimeofday(&t0, NULL);
	//for(int i = 0; i < 10; i++)
	for(;;)
	{
		double fr, fl, br, bl;
		fr = fl = br = bl = -1;
		if(!motordriver.getEncoderVel(fl,fr, bl, br))
		{
			timeval now;
			gettimeofday(&now, NULL);
			double t = double(now.tv_sec - t0.tv_sec) + ( double(1e-6)*double(now.tv_usec) - double(1e-6)*double(t0.tv_usec) );
			std::cout << "t: " << t << " motor" << stepmag << " fr: " << fr << " fl: " << fl << " br: " << br << " bl: " << bl << std::endl;
		}	
		usleep(1e6);
	}

}
