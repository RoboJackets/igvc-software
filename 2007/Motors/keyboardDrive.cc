#include <iostream>
#include "libplayerc++/playerc++.h"

int main(int argc, char *argv[])
{
	using namespace PlayerCc;

	PlayerClient	robot("localhost");
	Position2dProxy	pp(&robot,0);

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
