#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

class OSMC_driver
{

	public:
	OSMC_driver();

	#if 0
	bool connect();
	bool is_connected();
	#endif

	bool getCurrentData(current_reply_t& fwd, current_reply_t& aft);
	bool getJoystickData(joystick_reply_t& f, joystick_reply_t& b);

	//make 0 vel a nonrolling stop
	bool setmotorPWM(const byte rightDirFWD, const byte rightDutyCycleFWD, const byte leftDirFWD, const byte leftDutyCycleFWD, const byte rightDirAFT, const byte rightDutyCycleAFT, const byte leftDirAFT, const byte leftDutyCycleAFT);

	void getNewVel_dumb(const double rtarget, const double ltarget, const double rvel, const double lvel, const int rmset, const int lmset,  int& out_rmset, int& out_lmset);
	void getNewVel_pd(const double ltarget, const double rtarget, const double lvel, const double rvel,const double lastlvel, const double lastrvel, const int rmset, const int lmset, double dt, int& out_rmset, int& out_lmset);

	//follow path
	void followCirc(double radius, double vmag);

	//add these commands
	//bool enableMotor();
	//bool disableMotor();

	private:

	bool setmotorPWM(ArduinoInterface& ai, const byte rightDir, const byte rightDutyCycle, const byte leftDir, const byte leftDutyCycle);
	current_reply_t getCurrentData(ArduinoInterface& ai);
	joystick_reply_t getJoystickData(ArduinoInterface& ai);

	ArduinoInterface fwr;
	ArduinoInterface aft;

	const static int _max_speed_;
	const static int MINREQSPEED;

	bool m_connected;
};
