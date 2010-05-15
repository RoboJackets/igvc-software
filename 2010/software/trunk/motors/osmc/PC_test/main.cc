
#include <iostream>
#include <ctime>
#include "OSMC_driver.hpp"
//#include <>

//static const int speedsetdel = 3 * 1e4;
static const int speedsetdel = 2 * 1e4;

int main()
{
	OSMC_driver drive;


	for(;;)
	{
		for(int i = 0; i <= 255; i+=15)
		{
			//time_t t1 = time(NULL);
			double left, right;
			drive.setmotorPWM(MC_MOTOR_FORWARD, i, MC_MOTOR_FORWARD, i);
			drive.getEncoderVel(right, left);
			std::cout << "l: " << left << "right" << right << std::endl;
			usleep(speedsetdel);
			//time_t t2 = time(NULL);
			//double dt = ((double)t2 - (double)t1);
			std::cout <<"forward pwm: " << i << std::endl;
		}
		drive.setmotorPWM(MC_MOTOR_FORWARD, 255, MC_MOTOR_FORWARD, 255);

		usleep(2e6);
		for(int i = 255; i >= 0; i-=50)
		{
			double left, right;
			drive.setmotorPWM(MC_MOTOR_FORWARD, i, MC_MOTOR_FORWARD, i);
			drive.getEncoderVel(right, left);
			std::cout << "l: " << left << "right" << right << std::endl;
			usleep(speedsetdel);
			std::cout <<"forward pwm: " << i << std::endl;
		}
		drive.setmotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0);
		return 0;

	
		//clock_t t1 = clock();
		time_t t1 = time(NULL);
		for(int i = 255; i >= 0; i--)
		{

			drive.setmotorPWM(MC_MOTOR_FORWARD, i, MC_MOTOR_FORWARD, i);
			usleep(speedsetdel);
			std::cout <<"forward pwm: " << i << std::endl;
		}
			//clock_t t2 = clock();
		time_t t2 = time(NULL);
	#if 0
		for(int i = 0; i <= 255; i++)
		{
			//clock_t t1 = clock();
			drive.setmotorPWM(MC_MOTOR_REVERSE, i, MC_MOTOR_REVERSE, i);
			usleep(speedsetdel);
			std::cout <<"reverse pwm: " << i << std::endl;
			//clock_t t2 = clock();
			//std::cout <<"pwm: " << i << "\tdt: "<< (t2 - t1) / CLOCKS_PER_SEC << std::endl;
		}
		for(int i = 255; i >= 0; i--)
		{
			//clock_t t1 = clock();
			drive.setmotorPWM(MC_MOTOR_REVERSE, i, MC_MOTOR_REVERSE, i);
			std::cout <<"reverse pwm: " << i << std::endl;
			usleep(speedsetdel);
			//clock_t t2 = clock();
			//std::cout <<"pwm: " << i << "\tdt: "<< (t2 - t1) / CLOCKS_PER_SEC << std::endl;
		}
	#endif
	}
	usleep(1e6);
}
