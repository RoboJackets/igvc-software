#ifndef JOYSTICK_DRIVE_HPP
#define JOYSTICK_DRIVE_HPP

#include <SDL/SDL.h>
#include <cmath>

#include "motors_old.h"

class joystickDrive
{
	public:
		joystickDrive();
		~joystickDrive();

		inline double getHeading();
		void setMotor();
		void setMotorTank();

	private:
		Motors_Old m_motorCtr;
		pthread_t joystick_thread;
		SDL_Joystick *joystick;

		const char* joystick_open(void);
		void joystick_close(void);

		void readJoystick();

		int leftAnalogX;
		int leftAnalogY;
		int rightAnalogX;
		int rightAnalogY;
		int dPadX;
		int dPadY;
		int joystickButtons;
};

#endif
