#include "OSMC_driver.hpp"

#include <algorithm>

const int OSMC_driver::_max_speed_ = 130;
const int OSMC_driver::MINREQSPEED = 30;

OSMC_driver::OSMC_driver()
{
	m_connected = false;
	fwr.initLink(OSMC_IF_FOR_BOARD);
	aft.initLink(OSMC_IF_AFT_BOARD);
	m_connected = true;
}
#if 0
OSMC_driver::connect()
{
	ai.initLink(OSMC_IF_BOARD);
	m_connected = true;
}
#endif

bool OSMC_driver::getCurrentData(current_reply_t& f, current_reply_t& b)
{
	f = getCurrentData(fwr);
	b = getCurrentData(aft);
	return true;
}

current_reply_t OSMC_driver::getCurrentData(ArduinoInterface& ai)
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


bool OSMC_driver::setmotorPWM(const byte rightDirFWD, const byte rightDutyCycleFWD, const byte leftDirFWD, const byte leftDutyCycleFWD, const byte rightDirAFT, const byte rightDutyCycleAFT, const byte leftDirAFT, const byte leftDutyCycleAFT)
{
	bool ret = false;
	ret |= setmotorPWM(fwr, rightDirFWD, rightDutyCycleFWD, leftDirFWD, leftDutyCycleFWD);
	ret |= setmotorPWM(aft, rightDirAFT, rightDutyCycleAFT, leftDirAFT, leftDutyCycleAFT);

	return ret;
}

bool OSMC_driver::setmotorPWM(ArduinoInterface& ai, byte rightDir, byte rightDutyCycle, byte leftDir, byte leftDutyCycle)
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

bool OSMC_driver::getJoystickData(joystick_reply_t& f, joystick_reply_t& b)
{
	f = getJoystickData(fwr);
	b = getJoystickData(aft);
	return true;
}

joystick_reply_t OSMC_driver::getJoystickData(ArduinoInterface& ai)
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

// start dumb + hysteresis control code
void OSMC_driver::getNewVel_dumb(const double rtarget, const double ltarget, const double rvel, const double lvel, const int rmset, const int lmset,  int& out_rmset, int& out_lmset)
{
	int posstep = 1;
	int negstep = -1;

	double thresh = .03;

	double lerror = ltarget - lvel;
	double rerror = rtarget - rvel;

	if(lerror > thresh)
	{
		out_lmset = lmset + posstep;
	}
	else if(lerror < -thresh)
	{
		out_lmset = lmset + negstep;
	}
	else
	{
		out_lmset = lmset;
	}

	if(rerror > thresh)
	{
		out_rmset = rmset + posstep;
	}
	else if(rerror < -thresh)
	{
		out_rmset = rmset + negstep;
	}
	else
	{
		out_rmset = rmset;
	}
}

void OSMC_driver::getNewVel_pd(const double ltarget, const double rtarget, const double lvel, const double rvel,const double lastlvel, const double lastrvel, const int rmset, const int lmset, double dt, int& out_rmset, int& out_lmset)
{

	const double kp = .1;
	const double kd = .1;

	double lerror = ltarget - lvel;
	double rerror = rtarget - rvel;

	double lslope = (lvel - lastlvel) / dt;
	double rslope = (rvel - lastrvel) / dt;

	out_lmset = kp*lerror + kd*lslope;
	out_rmset = kp*rerror + kd*rslope;
}


void followCirc(double tangent[2], double vmag)
{

}
