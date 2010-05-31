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
	#endif
	#ifndef ENCODER_SIMULATE
		encoder = new quadCoderDriver();
	#else
		encoder = NULL;
	#endif
	m_connected = true;

	timeval now_t;
	gettimeofday(&now_t, NULL);
	t = double(now_t.tv_sec) + double(1e-6)*double(now_t.tv_usec);

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
	#endif
	#ifndef ENCODER_SIMULATE
		encoder = new quadCoderDriver(encoder_iface);
	#else
		encoder = NULL;
	#endif
	m_connected = true;

	timeval now_t;
	gettimeofday(&now_t, NULL);
	t = double(now_t.tv_sec) + double(1e-6)*double(now_t.tv_usec);

	set_motors(0, 0);
}
OSMC_driver::~OSMC_driver()
{
	set_motors(0, 0);
	delete encoder;
}
#if 0
OSMC_driver::connect()
{
	ai.initLink(OSMC_IF_BOARD);
	m_connected = true;
}
#endif

#ifndef ENCODER_SIMULATE
bool OSMC_driver::getEncoderData(new_encoder_pk_t& pk)
{
	return encoder->getEncoderState(pk);
}
#else
bool OSMC_driver::getEncoderData(new_encoder_pk_t& pk)
{
	return true;
}
#endif

#ifndef ENCODER_SIMULATE
bool OSMC_driver::getEncoderVel(double& rvel, double& lvel)
{
	return encoder->getEncoderVel(rvel, lvel);
}
#else
bool OSMC_driver::getEncoderVel(double& rvel, double& lvel)
{
	lvel = rvel = 0;
	return false;
}
#endif


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

bool OSMC_driver::set_motors(const int pwm)
{
	byte DutyCycle = std::min(abs(pwm), 255);
	byte Dir = (pwm < 0) ? MC_MOTOR_REVERSE : MC_MOTOR_FORWARD;

	return setMotorPWM(Dir, DutyCycle, Dir, DutyCycle);
}

bool OSMC_driver::set_motors(const int leftPWM, const int rightPWM)
{
	byte leftDutyCycle = std::min(abs(leftPWM), 255);
	byte leftDir = (leftPWM < 0) ? MC_MOTOR_REVERSE : MC_MOTOR_FORWARD;

	byte rightDutyCycle = std::min(abs(rightPWM), 255);
	byte rightDir = (rightPWM < 0) ? MC_MOTOR_REVERSE : MC_MOTOR_FORWARD;

	return setMotorPWM(rightDir, rightDutyCycle, leftDir, leftDutyCycle);
}

int OSMC_driver::set_heading(const int iFwdVelocity, const int iRotation)
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
bool OSMC_driver::setMotorPWM(const byte rightDir, const byte rightDutyCycle, const byte leftDir, const byte leftDutyCycle)
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
bool OSMC_driver::setMotorPWM(const byte rightDir, const byte rightDutyCycle, const byte leftDir, const byte leftDutyCycle)
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
	if((lvgoal == 0) && (rvgoal == 0))
	{
		return set_motors(0,0);
	}

	double now_rvel, now_lvel;
	if(getEncoderVel(now_rvel, now_lvel))
	{
		return true;
	}

	timeval now_t;
	gettimeofday(&now_t, NULL);
	double now_t_d = double(now_t.tv_sec) + double(1e-6)*double(now_t.tv_usec);
	double dt = now_t_d - t;

	int out_lmset, out_rmset;
	getNewVel_pd(now_lvel, now_rvel, dt, out_rmset, out_lmset);

	t = now_t_d;

	std::cout << "setr: " << out_rmset << " setl: " << out_lmset << std::endl;

	return set_motors(out_lmset, out_rmset);
}

//this won't change the sign of the pwm freq
void OSMC_driver::getNewVel_pd(const double now_lvel, const double now_rvel, const double dt, int& out_r, int& out_l)
{
	const double kp = 20;
	const double kd = 0;

	double lerror = lvgoal - now_lvel;
	double rerror = rvgoal - now_rvel;

	double lerror_slope = (lerror - last_l_error) / dt;
	double rerror_slope = (rerror - last_r_error) / dt;

	out_l = lpwm + kp*lerror + -kd*lerror_slope;
	out_r = rpwm + kp*rerror + -kd*rerror_slope;

	//persist the stuff that needs to be saved
	last_l_error = lerror;
	last_r_error = rerror;
}

void OSMC_driver::getLastPWMSent(byte& r, byte& l)
{
	r = rpwm;
	l = lpwm;
}

//set vel from vision vector
//linear map -- if x = 0, make both wheel same speed
//if angle = 90, go right, lock right wheel
//if angle = -90, go left, lock left wheel
//in m/s
bool OSMC_driver::set_vel_vec(const double y, const double x)
{
	if((y == 0) && (x == 0))
	{
		setVel_pd(0, 0);
		return set_motors(0,0);
	}

	double mag = hypot(y,x);
	double ang = M_PI / double(2) - atan2(y,x);
	double dir = (y > 0) ? 1 : -1;

	double adjslope = mag / (M_PI / double(2));

	std::cout << "mag: " << mag << " ang: " << ang << std::endl;

	double rdir = (mag - adjslope * ang) * dir;
	double ldir = (mag + adjslope * ang) * dir;

	setVel_pd(ldir, rdir);

	return false;
}
