
#include <iostream>
#include <ctime>
#include "OSMC_driver.hpp"
//#include <>

//static const int speedsetdel = 3 * 1e4;
static const int speedsetdel = 2 * 1e4;

int main()
{
	OSMC_driver drive;

	drive.setVel_pd(.5,.5);

	for(;;)
	{
		double left, right;
		byte lpwm, rpwm;

		drive.updateVel_pd();

		drive.getEncoderVel(right, left);
		drive.getLastPWMSent(rpwm, lpwm);
		std::cout << "left v: " << left << "right v: " << right << std::endl;
		std::cout << "left pwm: " << lpwm << "right pwm: " << rpwm << std::endl;

		usleep(speedsetdel);
	}
	drive.setmotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0);


}
