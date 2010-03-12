
#include <iostream>
#include <ctime>
#include "OSMC_driver.hpp"
#include "quadCoderDriver.hpp"
//#include <>

//static const int speedsetdel = 3 * 1e4;
static const int speedsetdel = 2 * 1e4;

static int rm = 0;
static int lm = 0;

const double ltarget = .5;
const double rtarget = .5;

int main()
{
	OSMC_driver motordriver;
	quadCoderDriver qD;

	motordriver.setmotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0);
	qD.resetCount();

	timeval t0;
	gettimeofday(&t0, NULL);
	for(int i = 0; i < 10; i++)
	{
		double r = -1, l = -1;
		if(qD.getEncoderVel(r,l))
		{
			std::cerr << "encoder fail" << std::endl;
			goto END;
		}
		int out_rmset, out_lmset;
		motordriver.getNewVel_dumb(ltarget, rtarget, l, r, lm, rm,  out_lmset, out_rmset);
		if(motordriver.setmotorPWM(MC_MOTOR_FORWARD, out_rmset, MC_MOTOR_FORWARD, out_lmset))
		{
			std::cerr << "motor set fail" << std::endl;
			goto END;
		}

		END:
		timeval now;
		gettimeofday(&now, NULL);
		double t = double(now.tv_sec - t0.tv_sec) + ( double(1e-6)*double(now.tv_usec - t0.tv_usec) );
		std::cout << "t: " << t << " motor r" << out_rmset << " motor l:" << out_lmset << " r: " << r << " l: " << l << std::endl;
		usleep(1e3);
	}

}
