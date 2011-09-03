
#include <iostream>
#include <ctime>
#include "OSMC_4wd_driver.hpp"

//static const int speedsetdel = 3 * 1e4;
static const int speedsetdel = 2 * 1e4;

static int rm = 0;
static int lm = 0;

const double ltarget = .5;
const double rtarget = .5;

int main()
{
	OSMC_4wd_driver motordriver;

	motordriver.setMotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0);

	timeval t0;
	gettimeofday(&t0, NULL);
	//for(int i = 0; i < 10; i++)
	for(;;)
	{
		//not implemented
		motordriver.getNewVel_dumb(rtarget, ltarget, r, l, rm, lm,  out_rmset, out_lmset);

		if(motordriver.setMotorPWM(MC_MOTOR_FORWARD, out_rmset, MC_MOTOR_FORWARD, out_lmset,MC_MOTOR_FORWARD, out_rmset, MC_MOTOR_FORWARD, out_lmset))
		{
			std::cerr << "motor set fail" << std::endl;
			goto END;
		}

		END:
		timeval now;
		gettimeofday(&now, NULL);
		double t = double(now.tv_sec - t0.tv_sec) + ( double(1e-6)*double(now.tv_usec - t0.tv_usec) );
		std::cout << "t: " << t << " motor r" << out_rmset << " motor l:" << out_lmset << " r: " << r << " l: " << l << std::endl;
		usleep(1e5);

		lm = out_lmset;
		rm = out_rmset;
	}

}
