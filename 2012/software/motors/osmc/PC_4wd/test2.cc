
#include <iostream>
#include <ctime>
#include "OSMC_4wd_driver.hpp"

static const int speedsetdel = 2 * 1e4;

int main()
{
	OSMC_4wd_driver drive;

	for(;;)
	{
		for(int i = 0; i <= 255; i+=15)
		{
			drive.setMotorPWM(MC_MOTOR_FORWARD, byte(i), MC_MOTOR_FORWARD, byte(i), MC_MOTOR_FORWARD, byte(i), MC_MOTOR_FORWARD, byte(i));
			usleep(speedsetdel);
			std::cout <<"forward pwm: " << i << std::endl;
		}
		drive.setMotorPWM(MC_MOTOR_FORWARD, 255, MC_MOTOR_FORWARD, 255, MC_MOTOR_FORWARD, 255, MC_MOTOR_FORWARD, 255);

		usleep(2e6);

		for(int i = 255; i >= 0; i-=50)
		{

			drive.setMotorPWM(MC_MOTOR_FORWARD, byte(i), MC_MOTOR_FORWARD, byte(i),MC_MOTOR_FORWARD, byte(i), MC_MOTOR_FORWARD, byte(i));
			usleep(speedsetdel);
			std::cout <<"forward pwm: " << i << std::endl;
		}
		drive.setMotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0,MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0);

		usleep(2e6);

		for(int i = 0; i <= 255; i+=15)
		{
			drive.setMotorPWM(MC_MOTOR_REVERSE, byte(i), MC_MOTOR_REVERSE, byte(i), MC_MOTOR_REVERSE, byte(i), MC_MOTOR_REVERSE, byte(i));
			usleep(speedsetdel);
			std::cout <<"forward pwm: " << i << std::endl;
		}
		drive.setMotorPWM(MC_MOTOR_REVERSE, 255, MC_MOTOR_REVERSE, 255, MC_MOTOR_REVERSE, 255, MC_MOTOR_REVERSE, 255);

		usleep(2e6);

		for(int i = 255; i >= 0; i-=50)
		{

			drive.setMotorPWM(MC_MOTOR_REVERSE, byte(i), MC_MOTOR_REVERSE, byte(i),MC_MOTOR_REVERSE, byte(i), MC_MOTOR_REVERSE, byte(i));
			usleep(speedsetdel);
			std::cout <<"forward pwm: " << i << std::endl;
		}
		drive.setMotorPWM(MC_MOTOR_REVERSE, 0, MC_MOTOR_REVERSE, 0,MC_MOTOR_REVERSE, 0, MC_MOTOR_REVERSE, 0);
	}
	usleep(1e6);
}
