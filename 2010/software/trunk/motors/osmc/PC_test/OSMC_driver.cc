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

void OSMC_driver::setmotorPWM(char rightDutyCycle, char leftDutyCycle)
{
	ai.sendCommand(MC_SET_RL_DUTY_CYCLE, NULL, 0);

	byte cmdresp;
	byte* data = NULL;
	ai.recvCommand(cmdresp, data);

	if(data != NULL)
	{
		delete[] data;
	}
}
