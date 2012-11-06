#include "OSMC_driver.hpp"
#include "XmlConfiguration.h"//for knowing whether to simulate motors

#include <algorithm>
#include <iostream>

const int OSMC_driver::_max_speed_ = 130;
const int OSMC_driver::MINREQSPEED = 30;

OSMC_driver::OSMC_driver()
{
	std::cout<<"WARNING: YOU SHOULD USE 4WD!"<<std::endl;
	std::cout.flush();
	OSMC_driver(OSMC_IF_BOARD, ENCODER_IF_BOARD);
	
	/*
	std::cout<<"WARNING: THIS CODE BELIEVED TO BE DEAD! WAS NOT UPDATED TO REFLECT MOTOR SIMULATION 3/24/12"<<std::endl;
	lvgoal = 0;
	rvgoal = 0;
	m_connected = false;
	encoder = NULL;
	#ifndef MOTOR_SIMULATE
		ai.initLink(OSMC_IF_BOARD);
	#endif
	#ifndef ENCODER_SIMULATE
		//encoder_l = new quadCoderDriver_signed(ENCODER_IF_AFT_LEFT_BOARD);
		//encoder_r = new quadCoderDriver_signed(ENCODER_IF_AFT_RIGHT_BOARD);
	#else
		encoder_l = NULL;
		encoder_r = NULL;
	#endif
	m_connected = true;

	timeval now_t;
	gettimeofday(&now_t, NULL);
	t = double(now_t.tv_sec) + double(1e-6)*double(now_t.tv_usec);

	set_motors(0, 0);*/
}

OSMC_driver::OSMC_driver(byte motor_iface, byte encoder_iface)
{
	/* load xml file to see if we need encoders/motors*/
	XmlConfiguration cfg("Config.xml");
	
	useMotors   = cfg.getInt("useMotors");
	useEncoders = cfg.getInt("useEncoders");
	useMagnetometer = cfg.getInt("useMagnetometer");
	
	lvgoal = 0;
	rvgoal = 0;
	m_connected = false;
	if(useMotors){
		ai.initLink(motor_iface);
	}
	if(useEncoders){
		encoder = new quadCoderDriver_signed(encoder_iface);
	}
	else{
		encoder = NULL;
	}
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
/*
#ifndef ENCODER_SIMULATE
bool OSMC_driver::getEncoderData(new_encoder_pk_t& pk)
{
	return encoder->getEncoderState(pk);

#else
bool OSMC_driver::getEncoderData(new_encoder_pk_t& pk)
{
	return true;
}
#endif
*/

bool OSMC_driver::getEncoderVel(double& lvel, double& rvel)
{
	//return encoder->getEncoderVel(rvel, lvel);
	bool ret = false;
	if(useEncoders){
		ret = encoder->getEncoderVel(lvel, rvel);
	}else{
		lvel = rvel = 0;
		return false;
	}
	return ret;
}

bool OSMC_driver::getEncoderDist(double& ldist, double& rdist)
{
	bool ret = false;
	if(useEncoders){
		ret = encoder->getEncoderDist(ldist, rdist);
	}else{
		ldist = rdist = 0;
		return false;
	}
	return ret;
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

#ifndef MOTOR_SIMULATE//deprecated, now using useMotors and useEncoders
bool OSMC_driver::setMotorPWM(const byte rightDir, const byte rightDutyCycle, const byte leftDir, const byte leftDutyCycle)
{
	
	//byte clamped_rightDutyCycle = (rightDutyCycle > 170) ? 170 : rightDutyCycle;
	//byte clamped_leftDutyCycle = (leftDutyCycle > 170) ? 170 : leftDutyCycle;
	byte clamped_rightDutyCycle = rightDutyCycle;
	byte clamped_leftDutyCycle = leftDutyCycle;

	speed_set_t cmdpk;
	cmdpk.sr = clamped_rightDutyCycle;
	cmdpk.rightDir = rightDir;
	cmdpk.sl = clamped_leftDutyCycle;
	cmdpk.leftDir = leftDir;

	std::cout << "r: ";
	if(rightDir == MC_MOTOR_REVERSE)
	{
		std::cout << -1*(int)rightDutyCycle;
	}
	else
	{
		std::cout << (int)rightDutyCycle;
	}
	std::cout << " l: ";
	if(leftDir == MC_MOTOR_REVERSE)
	{
		std::cout << -1*(int)leftDutyCycle;
	}
	else
	{
		std::cout << (int)leftDutyCycle;
	}
 	std::cout << std::endl;
	
	
	if(useMotors)if(ai.sendCommand(MC_SET_RL_DUTY_CYCLE, &cmdpk, sizeof(speed_set_t)))
	{
		return true;
	}

	byte cmdresp;
	byte* data = NULL;

	if(useMotors)if(ai.recvCommand(cmdresp, data))
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
#else //code should never run
bool OSMC_driver::setMotorPWM(const byte rightDir, const byte rightDutyCycle, const byte leftDir, const byte leftDutyCycle)
{
	std::cout<<"WARNING: THIS CODE BELIEVED TO BE DEAD! WAS NOT UPDATED TO REFLECT MOTOR SIMULATION 3/24/12"<<std::endl;
	int sr = (rightDir == MC_MOTOR_FORWARD) ? int(rightDutyCycle) : -int(rightDutyCycle);
	int sl = (leftDir == MC_MOTOR_FORWARD) ? int(leftDutyCycle) : -int(leftDutyCycle);

	//std::cout << "right: " << sr << "\tleft: " << sl << std::endl;

//	if((rightDir == MC_MOTOR_REVERSE) || (leftDir == MC_MOTOR_REVERSE))
//	{
//		//atach dbg here
//		std::cout << "robot is moving backwards!!!" << std::endl;
//	}

	return false;
}
#endif
joystick_reply_t OSMC_driver::getJoystickData()
{
	std::cout<<"WARNING: THIS CODE BELIEVED TO BE DEAD! WAS NOT UPDATED TO REFLECT MOTOR SIMULATION 3/24/12"<<std::endl;
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

/*
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
*/

/*
//set vel from vision vector
//jacob magic number algo
//linear map -- if x = 0, make both wheel same speed
//if angle = 90, go right, lock right wheel
//if angle = -90, go left, lock left wheel
//in pwm count
//input vector is unit vector
bool OSMC_driver::set_vel_vec(const double y, const double x)
{
	if((y == 0) && (x == 0))
	{
		setVel_pd(0, 0);
		return set_motors(0,0);
	}

	//double mag = 127;
	double mag = 70;
	//double asy = abs(y) / mag;
	//double asx = abs(x) / mag;
	double ang = M_PI / double(2) - atan2(y,x);
	double dir = (y > 0) ? 1 : -1;

	double adjslope = mag / (M_PI / double(2));

	std::cout << "mag: " << mag << " ang: " << ang << std::endl;

	double rdir = (mag - double(2)*double(1.5)*adjslope * ang) * dir;
	double ldir = (mag + double(2)*double(1.5)*adjslope * ang) * dir;

	return set_motors(ldir, rdir);
}
*/

//set vel from vision vector
//paul magic number algo
//linear map -- if x = 0, make both wheel same speed

//in pwm count
bool OSMC_driver::set_vel_vec(const double y, const double x)
{
	if((y == 0) && (x == 0))
	{
		setVel_pd(0, 0);
		return set_motors(0,0);
	}

	//const double fwdmag = 70;
	/*const double fwdmag = 60;
	const double dir = (y >= 0) ? 1 : -1;
	const double revmag = -30;
	const double turnthresh = M_PI / double(6);
	const double ang = M_PI / double(2) - atan2(y,x);
	const double forwardslope = fwdmag / (M_PI / double(2));
	const double backwardslope = -revmag / (M_PI / double(2));

	double rspeed, lspeed;
	std::string branch;
	if(ang < -turnthresh)
	{
		rspeed = (fwdmag - forwardslope * ang) * dir;
		lspeed = (revmag + backwardslope * (ang - turnthresh)) * dir;
		branch = "ang < -turnthresh";
	}
	else if(ang > turnthresh)
	{
		rspeed = (revmag - backwardslope * (ang - turnthresh)) * dir;
		lspeed = (fwdmag + forwardslope * ang) * dir;
		branch = "ang > turnthresh";
	}
	else 
	{
		rspeed = (fwdmag - forwardslope * ang) * dir;
		lspeed = (fwdmag + forwardslope * ang) * dir;
		branch = "abs(ang) < turnthresh";
	}*/
	//std::cout << "r: " << rspeed << " l: " << lspeed << "angle: " << ang << " branch: " << branch << std::endl;



	/************************************************************************
	This function calculates "good" values to send to the motors 
	when the robot wants to go in the direction specified
	by the vector (x,y), assuming that the robot is facing along the +y axis.
	
	Critical parameters: FWDMAG, DIFFMUL
	
	The function works in two stages: 
	1. The ideal velocities for the wheels are generated to correspond to the following motor curves:
	
		Left: 
			            ^
			            |[vel]
			            |
			  +FWDMAG ->|----------                                                  
			           /|                                                            
			          / |                                                            
			         /  |                                                            
			        /   |                                                            
		<----------/-------------------->                                            
			  |   /|    |    |    |     [deg cw]                                         
			 -90 /-45   |   +45  +90                                                 
			    /       |                                                            
			   /        |                                                            
			  /         |<- -FWDMAG                                                    
			            |                              
			            v
		 
		 Right:
			            ^
			            |[vel]
			            |
			  ----------|<- +FWDMAG                                                               
			            |\                                                           
			            | \                                                          
			            |  \                                                         
			            |   \                                                        
		<--------------------\---------->                                            
			  |    |    |    |\   |     [deg cw]                                         
			 -90  -45   |   +45\ +90                                                 
			            |       \                                                    
			            |        \                                                   
			  -FWDMAG ->|         \                                          
			            |                              
			            v
		 These were chosen so that full left and right would turn in place,
		 straight would go straight, and 45deg would lock up one side.
	 
	 2. The velocities are corrected to get the values needed to get those behaviors.
		 
		 Unfortunately the velocities produced by directly sending these 
		 signals don't have the desired behavior. For example, if the left wheel
		 is told to go 100 and the right 0, the left side goes slower and the 
		 right side moves forward some. In other words the left and right wheel 
		 behaviors are coupled so that try to behave similarly when told to do 
		 different things, they act more similarly than desired.
		 
		 To correct for the discrepancy, the difference in left and right wheel speed are amplified by DIFFMUL.
	     
	*******************************************************************************/


	
	const double fwdmag = 80;// this controls the basic speed the wheels run at
	const double diffmul = .2;// this is how much the difference in wheel powers is multiplied by to correct for velocity crosstalk
	const double dir = (y >= 0) ? 1 : -1;//behavior of this function not guaranteed to be smooth crossing to y<0;
	const double slope = 2.0 * fwdmag / (M_PI / 2.0);
	const double ang = M_PI / double(2) - atan2(y,x);
	double lspeed, rspeed;
	// Get ideal left and right speeds	
	if (ang < 0)
	{
		lspeed = (fwdmag + ang * slope) * dir;
		rspeed = fwdmag * dir;			
	}
	else
	{
		lspeed = fwdmag * dir;
		rspeed = (fwdmag - ang * slope) * dir;
	}
	
	// Amplify speed difference to correct for coupled motor behavior.
	double diff = rspeed - lspeed;
	rspeed = rspeed + diff*diffmul;
	lspeed = lspeed - diff*diffmul;

	return set_motors(lspeed, rspeed);
}

bool OSMC_driver::setLight(const byte option)
{
	if(useMotors){
		return ai.sendCommand(MC_SET_LIGHT,&option,1);
	}else{
		return false;
	}
}

bool OSMC_driver::GetMagnetometerHeading(int& heading, int& X_Value, int& Y_Value)
{
	if(ai.sendCommand(MAG_GET_MAGDATA, NULL, 0))
	{
		return true;
	}

	byte retcmd = MAG_GET_MAGDATA;
	byte* data = NULL;
	
	if(ai.recvCommand(retcmd, data))
	{
		return true;
	}

	magnetometer_pk_t out;
	if( retcmd!=MAG_GET_MAGDATA)
	{
		std::cout <<"not a magnetometer packet! got id:"<<(int)retcmd<<std::endl;
		return true;
	}
	
	memcpy(&out, data, sizeof(magnetometer_pk_t));

	heading = out.angle;
	heading += 360;
	heading %= 360;	
	X_Value = out.X_val;	
	Y_Value = out.Y_val;	
	delete[] data;
	return false;
}

