#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

class OSMC_driver
{

	public:
	OSMC_driver();

	reply_dtick_t getEncoderData();
	current_reply_t getCurrentData();
	joystick_reply_t getJoystickData();

	//make 0 vel a nonrolling stop
	bool setmotorPWM(byte rightDir, byte rightDutyCycle, byte leftDir, byte leftDutyCycle);

	//add these commands
	//bool enableMotor();
	//bool disableMotor();

	private:
	ArduinoInterface ai;
};
