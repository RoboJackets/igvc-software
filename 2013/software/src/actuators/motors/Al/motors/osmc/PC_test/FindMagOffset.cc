#include <iostream>
#include <ctime>
#include "OSMC_driver.hpp"

int main()
{
	OSMC_driver drive(OSMC_IF_FOR_BOARD, NULL);
	int angle;
	int XV;
	int YV;
	
	std::cout << "This file is to find the appropriate offsets for the magnetometer. Once the function begins, move the magnetometer, slowly, 360 degrees twice to get a full range of data. Press enter to begin.";
	char ent = ' ';
	std::cin >> ent;
	for(int x = 0; x <= 5; x++)
	{
		drive.GetMagnetometerHeading(angle, XV, YV);
	}
	
	int X_max = -999;
	int X_min = 999;
	int Y_max = -999;
	int Y_min = 999;
	
	int X_Offset = 0;
	int Y_Offset = 0;
	for(;;)
	{
		drive.GetMagnetometerHeading(angle, XV, YV);
		std::cout << "\t\tcurrx:"<<XV<<"  curry:"<<YV<<std::endl;
		if(XV>X_max){X_max=XV;}
		if(XV<X_min){X_min=XV;}
		if(YV>Y_max){Y_max=YV;}
		if(YV<Y_min){Y_min=YV;}
		std::cout << "X Max:" << X_max << "\n";
		std::cout << "X Min:" << X_min << "\n";
		std::cout << "Y Max:" << Y_max << "\n";
		std::cout << "Y Min:" << Y_min << "\n";	
		X_Offset = (X_min - X_max)/2 - X_min;
		Y_Offset = (Y_min - Y_max)/2 - Y_min;
		std::cout << "X Offset:" << X_Offset << "\n";
		std::cout << "Y Offset:" << Y_Offset << "\n";
		std::cerr<<XV<<"\t"<<YV<<std::endl;
	}
}
