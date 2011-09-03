#ifndef JOYSTICK_DRIVE_HPP
#define JOYSTICK_DRIVE_HPP

#include <SDL/SDL.h>
#include <cmath>

//#include "OSMC_driver.hpp"
#include "OSMC_4wd_driver.hpp"
//#include "quadCoderDriver.hpp"
//#define TEBOARD_MOTOR_CTRL
#define ARDUINO_MOTOR_CTRL

typedef enum {
BUTTON1 = 0, BUTTON2 = 1, BUTTON3 = 2, BUTTON4 = 3, 
BUTTON5 = 4, LT_Trig = 4, button6 = 5, RT_TRIG = 5,
BUTTON7 = 6, LB_TRIG = 6, BUTTON8 = 7, RB_TRIG = 7,
BUTTON9 = 8, BUTTON10 = 9
} BUTTON;

class joystickDrive
{
	public:
		joystickDrive();
		joystickDrive(OSMC_4wd_driver*);
		~joystickDrive();

		inline double getHeading();
		void setMotor();
		void setMotorTank();
		bool shouldQuit();
		bool manualOverride();
		void readJoystick();
		void printLoop();

	private:

		//OSMC_driver* m_motorCtr;
		OSMC_4wd_driver* m_motorCtr;
		//quadCoderDriver qD;

		pthread_t joystick_thread;
		SDL_Joystick *joystick;

		const char* joystick_open(void);
		void joystick_close(void);
		bool check_button(BUTTON);

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
