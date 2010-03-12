
#include "quadCoderDriver.hpp"

/* ROBOT PHYSICAL DEFINITIONS */  /*TODO: determine these values */
#define MOTOR_RATIO		((double)20)
//#define WHEEL_RADIUS		((double)5 / (double)MOTOR_RATIO)
#define WHEEL_RADIUS		(double(.146) / double(2))
#define WHEEL_BASE		((double)28)
#define NUMBERTICKS		double(800)

#define SMAPLEDELAY		double(5e-3)
quadCoderDriver::quadCoderDriver()
{
	m_connected = false;
	ai.initLink(ENCODER_IF_BOARD);
	m_connected = true;
}

bool quadCoderDriver::getEncoderState(new_encoder_pk_t& out)
{

	if(ai.sendCommand(ENCODER_GET_READING, NULL, 0))
	{
		return true;
	}

	byte retcmd = ENCODER_GET_READING;
	byte* data = NULL;
	
	if(ai.recvCommand(retcmd, data))
	{
		return true;
	}

	memcpy(&out, data, sizeof(new_encoder_pk_t));
	delete[] data;
	return false;
}

bool quadCoderDriver::getEncoderVel(double& rvel, double& lvel)
{
	new_encoder_pk_t out;
	
	if(getEncoderState(out))
	{
		return true;
	}

	double ldtheta = ((out.dl / NUMBERTICKS * double(2)*M_PI) / MOTOR_RATIO) / SMAPLEDELAY;
	double rdtheta = ((out.dr / NUMBERTICKS * double(2)*M_PI) / MOTOR_RATIO) / SMAPLEDELAY;

	lvel = ldtheta * WHEEL_RADIUS;
	rvel = rdtheta * WHEEL_RADIUS;
}

bool quadCoderDriver::resetCount()
{
	new_encoder_pk_t out;
	
	if(ai.sendCommand(ENCOER_RESET_COUNT, NULL, 0))
	{
		return true;
	}

	byte retcmd = ENCODER_GET_READING;
	byte* data = NULL;

	return ai.recvCommand(retcmd, data);
}
