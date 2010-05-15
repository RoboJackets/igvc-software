#include "OSMC_driver.hpp"

#include <algorithm>

const int OSMC_driver::_max_speed_ = 130;
const int OSMC_driver::MINREQSPEED = 30;

OSMC_driver::OSMC_driver()
{
	lvgoal = 0;
	rvgoal = 0;
	m_connected = false;
	encoder = NULL;
	#ifndef MOTOR_SIMULATE
	ai.initLink(OSMC_IF_BOARD);
	encoder = new quadCoderDriver();
	#endif
	m_connected = true;
	set_motors(0, 0);
}

OSMC_driver::OSMC_driver(byte motor_iface, byte encoder_iface)
{
	lvgoal = 0;
	rvgoal = 0;
	m_connected = false;
	encoder = NULL;
	#ifndef MOTOR_SIMULATE
	ai.initLink(motor_iface);
	encoder = new quadCoderDriver(encoder_iface);
	#endif
	m_connected = true;
	set_motors(0, 0);
}
OSMC_driver::~OSMC_driver()
{
	delete encoder;
}
#if 0
OSMC_driver::connect()
{
	ai.initLink(OSMC_IF_BOARD);
	m_connected = true;
}
#endif
bool OSMC_driver::getEncoderData(new_encoder_pk_t& pk)
{
	return encoder->getEncoderState(pk);
}

bool OSMC_driver::getEncoderVel(double& rvel, double& lvel)
{
	return encoder->getEncoderVel(rvel, lvel);
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

	ldir = leftDir;
	lpwm = leftDutyCycle;
	rdir = rightDir;
	rpwm = rightDutyCycle;

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

void OSMC_driver::setVel_pd(double left, double right)
{
	lvgoal = left;
	rvgoal = right;
}

bool OSMC_driver::updateVel_pd()
{
	double now_rvel, now_lvel;
	timeval now_t;
	gettimeofday(&now_t, NULL);

	encoder->getEncoderVel(now_rvel, now_lvel);

	double now_t_d = double(now_t.tv_sec) + double(1e-6)*double(now_t.tv_usec);
	double dt = now_t_d - t;

	int out_lmset, out_rmset;
	getNewVel_pd(now_lvel, now_rvel, dt, out_rmset, out_lmset);

	t = now_t_d;

	return set_motors(out_lmset, out_rmset);
}

//this won't change the sign of the pwm freq
void OSMC_driver::getNewVel_pd(const double now_lvel, const double now_rvel, const double dt, int& out_r, int& out_l)
{
	const double kp = .1;
	const double kd = .1;

	double lerror = lvgoal - now_lvel;
	double rerror = rvgoal - now_rvel;

	double lerror_slope = (lerror - last_l_error) / dt;
	double rerror_slope = (rerror - last_r_error) / dt;

	out_l = lpwm + -kp*lerror + -kd*lerror_slope;
	out_r = rpwm + -kp*rerror + -kd*rerror_slope;

	//persist the stuff that needs to be saved
	last_l_error = lerror;
	last_r_error = rerror;
}

void OSMC_driver::getLastPWMSent(byte& r, byte& l)
{
	r = rpwm;
	l = lpwm;
}
