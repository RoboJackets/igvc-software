#ifndef JOYSTICK_DRIVE_HPP
#define JOYSTICK_DRIVE_HPP

#include <SDL/SDL.h>
#include <cmath>

#include "OSMC_driver.hpp"
//#include "quadCoderDriver.hpp"
//#define TEBOARD_MOTOR_CTRL
#define ARDUINO_MOTOR_CTRL

class joystickDrive
{
	public:
		joystickDrive();
		~joystickDrive();

		inline double getHeading();
		void setMotor();
		void setMotorTank();
		bool shouldQuit();

	private:

		OSMC_driver m_motorCtr;
		//quadCoderDriver qD;

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
		volatile bool quit;
};

#endif
