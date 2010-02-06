#include "OSMC_driver.hpp"

OSMC_driver::OSMC_driver()
{
	ai.initLink(OSMC_IF_BOARD);
}

reply_dtick_t OSMC_driver::getEncoderData()
{
	ai.sendCommand(MC_GET_ENCODER_TICK, NULL, 0);

	byte retcmd = 0;
	byte* data = NULL;
	ai.recvCommand(retcmd, data);

	reply_dtick_t out;
	
	memcpy(&out, data, sizeof(reply_dtick_t));
	delete[] data;
	return out;
}

current_reply_t OSMC_driver::getCurrentData()
{
	ai.sendCommand(MC_GET_RL_CURR_VAL, NULL, 0);

	byte retcmd = 0;
	byte* data = NULL;
	ai.recvCommand(retcmd, data);

	current_reply_t out;
	
	memcpy(&out, data, sizeof(current_reply_t));
	delete[] data;
	return out;
}

bool OSMC_driver::setmotorPWM(byte rightDir, byte rightDutyCycle, byte leftDir, byte leftDutyCycle)
{
	speed_set_t cmdpk;
	cmdpk.sr = rightDutyCycle;
	cmdpk.rightDir = rightDir;
	cmdpk.sl = leftDutyCycle;
	cmdpk.leftDir = leftDir;

	std::cout << "r: " << (int)rightDutyCycle << "l: " << (int)leftDutyCycle << std::endl;

	if(ai.sendCommand(MC_SET_RL_DUTY_CYCLE, &cmdpk, sizeof(speed_set_t)))
	{
		return true;
	}

	byte cmdresp;
	byte* data = NULL;

	if(ai.recvCommand(cmdresp, data))
	{
		return true;
	}

	if(data != NULL)
	{
		delete[] data;
	}
	return false;
}

joystick_reply_t OSMC_driver::getJoystickData()
{

	ai.sendCommand(MC_GET_JOYSTICK, NULL, 0);

	byte retcmd = 0;
	byte* data = NULL;
	ai.recvCommand(retcmd, data);

	joystick_reply_t out;
	
	memcpy(&out, data, sizeof(current_reply_t));
	delete[] data;
	return out;
}
