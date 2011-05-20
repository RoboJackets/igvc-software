#include "joystickDrive.hpp"

//Main now in joystickmain
int main()
{
	joystickDrive jD;

	for(;;)
	{
		//jD.printLoop();		
		jD.setMotor();
		jD.readJoystick();
		if(jD.shouldQuit())
		{
			return(0);
		}
		usleep(4e4);
		
	}
}

