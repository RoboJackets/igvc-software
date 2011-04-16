#include <iostream>
#include <ctime>
#include "OSMC_4wd_driver.hpp"

int main()
{
	OSMC_4wd_driver drive;
	bool error = drive.setLight(MC_LIGHT_STEADY);
	if (error)
		std::cout << "Error in function\n";
	else 
		std::cout << "Light steady\n";
	usleep(5e6);
	error = drive.setLight(MC_LIGHT_PULSING);
	if (error)
		std::cout << "Error in function\n";
	else 
		std::cout << "Light pulsing\n";		
}
