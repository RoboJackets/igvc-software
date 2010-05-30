
#include "OSMC_4wd_driver.hpp"

OSMC_4wd_driver::OSMC_4wd_driver() : FOR(OSMC_IF_FOR_BOARD, ENCODER_IF_FOR_BOARD) , AFT(OSMC_IF_AFT_BOARD, ENCODER_IF_AFT_BOARD)
{
	m_connected = true;
}

OSMC_4wd_driver::OSMC_4wd_driver(const byte FORosmc, const byte FORcoder, const byte AFTosmc, const byte AFTcoder) : FOR(FORosmc, FORcoder) , AFT(AFTosmc, AFTcoder)
{
	m_connected = true;
}

bool OSMC_4wd_driver::setMotorPWM(const byte FRdir, const byte FRmag, const byte FLdir, const byte FLmag, const byte BRdir, const byte BRmag, const byte BLdir, const byte BLmag)
{
	bool a = FOR.setMotorPWM(FRdir, FRmag, FLdir, FLmag);
	bool b = AFT.setMotorPWM(BRdir, BRmag, BLdir, BLmag);

	return a || b;
}

bool OSMC_4wd_driver::set_motors(const int pwm)
{
	bool a = FOR.set_motors(pwm);
	bool b = AFT.set_motors(pwm);

	return a || b;
}

bool OSMC_4wd_driver::setMotorPWM(const int FR, const int FL, const int BR, const int BL)
{
	byte FRdir = (FR >= 0) ? MC_MOTOR_FORWARD : MC_MOTOR_REVERSE;
	byte FLdir = (FL >= 0) ? MC_MOTOR_FORWARD : MC_MOTOR_REVERSE;
	byte BRdir = (BR >= 0) ? MC_MOTOR_FORWARD : MC_MOTOR_REVERSE;
	byte BLdir = (BL >= 0) ? MC_MOTOR_FORWARD : MC_MOTOR_REVERSE;

	int aFR = abs(FR);
	int aFL = abs(FL);
	int aBR = abs(BR);
	int aBL = abs(BL);

	byte FRmag = (aFR > 255) ? 255 : aFR;
	byte FLmag = (aFL > 255) ? 255 : aFL;
	byte BRmag = (aBR > 255) ? 255 : aBR;
	byte BLmag = (aBL > 255) ? 255 : aBL;

	return setMotorPWM(FRdir, FRmag, FLdir, FLmag, BRdir, BRmag, BLdir, BLmag);
}

void OSMC_4wd_driver::setMotorVel_pd(const double FR, const double FL, const double BR, const double BL)
{
	FOR.setVel_pd(FL, FR);
	AFT.setVel_pd(BL, BR);
}

bool OSMC_4wd_driver::updateVel_pd()
{
	bool a = FOR.updateVel_pd();
	bool b = AFT.updateVel_pd();

	return a || b;
}

bool OSMC_4wd_driver::getEncoderVel(double& FL, double& FR, double& BL, double& BR)
{
	bool a = FOR.getEncoderVel(FL, FR);
	bool b = AFT.getEncoderVel(BL, BR);
	
	return a || b;
}

bool OSMC_4wd_driver::set_vel_vec(const double y, const double x)
{
	bool a = FOR.set_vel_vec(y,x);
	bool b = AFT.set_vel_vec(y,x);

	return a || b;
}
