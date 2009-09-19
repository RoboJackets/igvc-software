
#include <iostream>
#include <ctime>
#include "OSMC_driver.hpp"
//#include <>

static const int speedsetdel = 5 * 1e4;

int main()
{
	OSMC_driver drive;

	for(;;)
	{
		reply_dtick_t datacoder = drive.getEncoderData();
	
		std::cout << "test coder" << std::endl;
	//std::cout << std::hex;
		std::cout << "data.dl:" << datacoder.dl << std::endl;
		std::cout << "data.dr:" << datacoder.dr << std::endl;
		std::cout << "data.dt:" << datacoder.dt << std::endl;
		usleep(1e5);
	}
/*
	for(;;)
	{
		current_reply_t datacurr = drive.getCurrentData();	
		std::cout << "test current" << std::endl;
		//std::cout << std::hex;
		std::cout << "data.il:" << datacurr.il << std::endl;
		std::cout << "data.ir:" << datacurr.ir << std::endl;
		usleep(1e5);
	}
*/

	for(;;)
	{
		joystick_reply_t datajoy = drive.getJoystickData();	
		std::cout << "test joy" << std::endl;
		//std::cout << std::hex;
		std::cout << "data.x:" << datajoy.joy_x << std::endl;
		std::cout << "data.y:" << datajoy.joy_y << std::endl;
		usleep(1e5);
	}

//	drive.setmotorPWM(MC_MOTOR_FORWARD, 255, MC_MOTOR_FORWARD, 255);
//	for(;;)
//	{
//		usleep(100);
//	}

	for(;;)
{
	for(int i = 0; i <= 255; i++)
	{
//		time_t t1 = time(NULL);
		drive.setmotorPWM(MC_MOTOR_FORWARD, i, MC_MOTOR_FORWARD, 0);
		usleep(speedsetdel);
//		time_t t2 = time(NULL);
//		double dt = ((double)t2 - (double)t1);
		std::cout <<"forward pwm: " << i << std::endl;
	}
		//clock_t t1 = clock();
	time_t t1 = time(NULL);
	for(int i = 255; i >= 0; i--)
	{

		drive.setmotorPWM(MC_MOTOR_FORWARD, i, MC_MOTOR_FORWARD, 0);
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
//		clock_t t2 = clock();
	//	std::cout <<"pwm: " << i << "\tdt: "<< (t2 - t1) / CLOCKS_PER_SEC << std::endl;
	}
	for(int i = 255; i >= 0; i--)
	{
//		clock_t t1 = clock();
		drive.setmotorPWM(MC_MOTOR_REVERSE, i, MC_MOTOR_REVERSE, i);
		std::cout <<"reverse pwm: " << i << std::endl;
		usleep(speedsetdel);
//		clock_t t2 = clock();
	//	std::cout <<"pwm: " << i << "\tdt: "<< (t2 - t1) / CLOCKS_PER_SEC << std::endl;
	}
#endif
}
	usleep(1e6);
}
