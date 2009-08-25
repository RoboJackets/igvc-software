
#include <iostream>
#include "OSMC_driver.hpp"
//#include <>
int main()
{
	OSMC_driver drive;

//for(;;)
{
	reply_dtick_t datacoder = drive.getEncoderData();
	current_reply_t datacurr = drive.getCurrentData();

	std::cout << "test coder" << std::endl;
	std::cout << std::hex;
	std::cout << "data.dl:" << datacoder.dl << std::endl;
	std::cout << "data.dr:" << datacoder.dr << std::endl;
	std::cout << "data.dt:" << datacoder.dt << std::endl;

	std::cout << "test current" << std::endl;
	std::cout << std::hex;
	std::cout << "data.il:" << datacurr.il << std::endl;
	std::cout << "data.ir:" << datacurr.ir << std::endl;

	for(char i = 0; i < 120; i++)
	{
		drive.setmotorPWM(i, i);
		usleep(5*1e4);
		std::cout << i << std::endl;
	}

	usleep(1e6);
}

	usleep(2 * 1e6);
}
