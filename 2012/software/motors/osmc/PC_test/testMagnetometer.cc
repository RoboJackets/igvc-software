#include <iostream>
#include <ctime>
#include "OSMC_driver.hpp"

int main()
{
	OSMC_driver drive;

	int angle;
	int XV;
	int YV;
	for (;;)
	{
		if (drive.GetMagnetometerHeading(angle, XV, YV))
		{
			std::cout << "Error in function\n";
		}

		else
		{
			std::cout << "Angle: " << angle << "\n";
			std::cout << "X: " << XV << "\n";
			std::cout << "Y: " << YV << "\n";	
		}	
		usleep(1000);
	}
}
