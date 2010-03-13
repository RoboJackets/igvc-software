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

	reply_dtick_t getEncoderData();
	current_reply_t getCurrentData();
	joystick_reply_t getJoystickData();

	//make 0 vel a nonrolling stop
	bool setmotorPWM(byte rightDir, byte rightDutyCycle, byte leftDir, byte leftDutyCycle);

	//shims from old motor
	bool set_motors(int leftVelocity, int rightVelocity);
	int set_heading(int iFwdVelocity, int iRotation);

	void getNewVel_dumb(const double rtarget, const double ltarget, const double rvel, const double lvel, const int rmset, const int lmset,  int& out_rmset, int& out_lmset);

	//add these commands
	//bool enableMotor();
	//bool disableMotor();

	private:
	ArduinoInterface ai;

	const static int _max_speed_;
	const static int MINREQSPEED;

	bool m_connected;
};
