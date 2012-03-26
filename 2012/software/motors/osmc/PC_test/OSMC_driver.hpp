#ifndef OSMC_DRIVER
#define OSMC_DRIVER

#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

#include "quadCoderDriver_signed.hpp"

//if this is defined, nothing is sent to the motors!, it is just printed to stdout
//#define MOTOR_SIMULATE 1
//#define ENCODER_SIMULATE 1
//#define CURRENT_SIMULATE 1

//following gnu libc tradition,
//a return value of false is success (eg, code 0)
//a return value of true is failure (eg, code nonzero)

class OSMC_driver
{

	public:
	OSMC_driver();
	OSMC_driver(byte motor_iface, byte encoder_iface);
	~OSMC_driver();

	#if 0
	bool connect();
	bool is_connected();
	#endif

	current_reply_t getCurrentData();
	joystick_reply_t getJoystickData();

	//make 0 vel a nonrolling stop
	bool setMotorPWM(const byte rightDir, const byte rightDutyCycle, const byte leftDir, const byte leftDutyCycle);

	//shims from old motor
	bool set_motors(const int leftPWM, const int rightPWM);
	bool set_motors(const int pwm);
	int set_heading(const int iFwdVelocity, const int iRotation);

	void getNewVel_dumb(const double rtarget, const double ltarget, const double rvel, const double lvel, const int rmset, const int lmset, int& out_rmset, int& out_lmset);

	//follow path
	void followCirc(double radius, double vmag);

	//add these commands
	//bool enableMotor();
	//bool disableMotor();

	//interact with the associated encoder
	bool getEncoderVel(double& lvel, double& rvel);
	bool getEncoderDist(double& ldist, double& rdist);
	//bool getEncoderDist(double&, double&);
	//bool getEncoderData(new_encoder_pk_t& pk);

	//run one pd loop
	void setVel_pd(double left, double right);
	bool updateVel_pd();

	//get state
	void getVelSet_pd();
	void getLastPWMSent(byte& r, byte& l);

	//set vel from vision vector
	bool set_vel_vec(const double y, const double x);

	//control for status light
	bool setLight(const byte option);
	bool GetMagnetometerHeading(int& heading, int& X_Value, int& Y_Value);

	private:
	//Whether to use motors or encoders, or fake it
	bool useMotors;
	bool useEncoders;
	
	ArduinoInterface ai;

	quadCoderDriver_signed* encoder;

	const static int _max_speed_;
	const static int MINREQSPEED;

	bool m_connected;

	//the pwm settings
	byte ldir, rdir;
	byte lpwm, rpwm;

	//left and right vels for pd loop
	double lvgoal;// m/s desired
	double rvgoal;// m/s desired
	double last_l_error;// m/s error at time t
	double last_r_error;// m/s error at time t
	double t;// seconds since unix epoch
	void getNewVel_pd(const double now_lvel, const double now_rvel, const double dt, int& out_r, int& out_l);
	static void loadConfig();//load config file to see if we need to use motors or encoders
};

#endif
