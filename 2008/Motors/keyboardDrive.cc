#include <iostream>
#include "libplayerc++/playerc++.h"

int main(int argc, char *argv[])
{
	using namespace PlayerCc;

	PlayerClient	robot("localhost");
	printf("connected to robot\n");
	Position2dProxy	pp(&robot,0);
	printf("connected to motors\n");

	for(;;)
	{
		double dLeftVelocity, dRightVelocity;
		std::cout << std::endl << "Left Velocity: ";
		std::cin >> dLeftVelocity;
		std::cout << "Right Velocity: ";
		std::cin >> dRightVelocity;
		pp.SetSpeed(dLeftVelocity, dRightVelocity);
	}
}
