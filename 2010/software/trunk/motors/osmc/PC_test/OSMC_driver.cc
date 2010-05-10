#include "OSMC_driver.hpp"

#include <algorithm>

const int OSMC_driver::_max_speed_ = 130;
const int OSMC_driver::MINREQSPEED = 30;

OSMC_driver::OSMC_driver()
{
	m_connected = false;
	#ifndef MOTOR_SIMULATE
	ai.initLink(OSMC_IF_BOARD);
	#endif
	m_connected = true;
}
#if 0
OSMC_driver::connect()
{
	ai.initLink(OSMC_IF_BOARD);
	m_connected = true;
}
#endif
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

bool OSMC_driver::set_motors(int leftVelocity, int rightVelocity)
{

	byte leftDutyCycle = std::min(abs(leftVelocity), 255);
	byte leftDir = (leftVelocity < 0) ? MC_MOTOR_REVERSE : MC_MOTOR_FORWARD;

	byte rightDutyCycle = std::min(abs(rightVelocity), 255);
	byte rightDir = (rightVelocity < 0) ? MC_MOTOR_REVERSE : MC_MOTOR_FORWARD;

	return setmotorPWM(rightDir, rightDutyCycle, leftDir, leftDutyCycle);
}

int OSMC_driver::set_heading(int iFwdVelocity, int iRotation)
{
	// convert
	int left  = iFwdVelocity + iRotation ;
	int right = iFwdVelocity - iRotation ;

	if (true)
	{
		// scale speed
		left  = int( float(left)  * float(_max_speed_) / float(255) ) ;
		right = int( float(right) * float(_max_speed_) / float(255) ) ;
	}
	else
	{
		// cap speed
		if (left  > _max_speed_) left  = _max_speed_ ;
		if (right > _max_speed_) right = _max_speed_ ;
	}

	// motors don't respond until certain output is reached
	if (right != 0) right += MINREQSPEED ;
	if (left  != 0) left  += MINREQSPEED ;

	// do it!
	return this->set_motors( left , right );
}

#ifndef MOTOR_SIMULATE
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
#else
bool OSMC_driver::setmotorPWM(byte rightDir, byte rightDutyCycle, byte leftDir, byte leftDutyCycle)
{
	int sr = (rightDir == MC_MOTOR_FORWARD) ? int(rightDutyCycle) : -int(rightDutyCycle);
	int sl = (leftDir == MC_MOTOR_FORWARD) ? int(leftDutyCycle) : -int(leftDutyCycle);

	std::cout << "right: " << sr << "\tleft: " << sl << std::endl;

	if((rightDir == MC_MOTOR_REVERSE) || (leftDir == MC_MOTOR_REVERSE))
	{
		//atach dbg here
		std::cout << "robot is moving backwards!!!" << std::endl;
	}

	return false;
}
#endif
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
