#include <iostream>
#include <ctime>
#include "OSMC_driver.hpp"

int main()
{
	OSMC_driver drive;

	int angle;

	if (drive.GetMagnetometerHeading(angle))
	{
		std::cout << "Error in function\n";
	}

	else
	{
		std::cout << angle << "\n";	
	}	
	usleep(1000);
}
