
#include <iostream>
#include <ctime>
#include "OSMC_driver.hpp"
//#include <>

//static const int speedsetdel = 3 * 1e4;
static const int speedsetdel = 1 * 1e5;

int main()
{
	OSMC_driver drive;

	drive.setMotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0);

	for(;;)
	{
		for(int i = 0; i <= 255; i+=1)
		{
			double left, right;
			drive.setMotorPWM(MC_MOTOR_FORWARD, i, MC_MOTOR_FORWARD, i);
			drive.getEncoderVel(right, left);
			std::cout << "l: " << left << "right" << right << std::endl;
			usleep(speedsetdel);
			std::cout <<"forward pwm: " << i << std::endl;
		}
		drive.setMotorPWM(MC_MOTOR_FORWARD, 255, MC_MOTOR_FORWARD, 255);

		usleep(2e6);
		for(int i = 255; i >= 0; i-=1)
		{
			double left, right;
			drive.setMotorPWM(MC_MOTOR_FORWARD, i, MC_MOTOR_FORWARD, i);
			drive.getEncoderVel(right, left);
			std::cout << "l: " << left << "right" << right << std::endl;
			usleep(speedsetdel);
			std::cout <<"forward pwm: " << i << std::endl;
		}
		drive.setMotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0);
	#if 1
		for(int i = 0; i <= 255; i++)
		{
			//clock_t t1 = clock();
			drive.setMotorPWM(MC_MOTOR_REVERSE, i, MC_MOTOR_REVERSE, i);
			usleep(speedsetdel);
			std::cout <<"reverse pwm: " << i << std::endl;
		}
		for(int i = 255; i >= 0; i--)
		{
			//clock_t t1 = clock();
			drive.setMotorPWM(MC_MOTOR_REVERSE, i, MC_MOTOR_REVERSE, i);
			std::cout <<"reverse pwm: " << i << std::endl;
			usleep(speedsetdel);
		}
	#endif
	}
	usleep(1e6);
}
