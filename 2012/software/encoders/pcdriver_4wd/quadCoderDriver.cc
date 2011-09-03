
#include "quadCoderDriver.hpp"

/* ROBOT PHYSICAL DEFINITIONS */  /*TODO: determine these values */
#define MOTOR_RATIO		((double)20)
//#define WHEEL_RADIUS		((double)5 / (double)MOTOR_RATIO)

//in meters
#define WHEEL_RADIUS		(double(.146) / double(2))
//unknown units
#define WHEEL_BASE		((double)28)

//there are 4 ticks per mark on the disk
//#define NUMBERTICKS		double(800)
#define NUMBERTICKS		double(400)

//this is det by delay within arduino code
#define SMAPLEDELAY		double(5e-3)
quadCoderDriver::quadCoderDriver()
{
	m_connected = false;
	fwd.initLink(ENCODER_IF_FOR_BOARD);
	aft.initLink(ENCODER_IF_AFT_BOARD);
	m_connected = true;
}

bool quadCoderDriver::getEncoderState(ArduinoInterface& ai, new_encoder_pk_t& out)
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

bool quadCoderDriver::getEncoderVel(double& rvelFWD, double& lvelFWD,double& rvelAFT, double& lvelAFT)
{
	bool a = getEncoderVel(fwd, rvelFWD, lvelFWD);
	bool b = getEncoderVel(aft, rvelAFT, lvelAFT);
	return a&b;
}

//in meters per second
bool quadCoderDriver::getEncoderVel(ArduinoInterface& ai, double& rvel, double& lvel)
{
	new_encoder_pk_t out;
	
	if(getEncoderState(ai, out))
	{
		return true;
	}

	double ldtheta = ((out.dl / NUMBERTICKS * double(2)*M_PI) / MOTOR_RATIO) / SMAPLEDELAY;
	double rdtheta = ((out.dr / NUMBERTICKS * double(2)*M_PI) / MOTOR_RATIO) / SMAPLEDELAY;

	lvel = ldtheta * WHEEL_RADIUS;
	rvel = rdtheta * WHEEL_RADIUS;

	return false;
}

bool quadCoderDriver::resetCount(byte iface)
{
	if(iface == ENCODER_IF_FOR_BOARD)
	{
		return resetCount(fwd);
	}
	else if(iface == ENCODER_IF_AFT_BOARD)
	{
		return resetCount(aft);
	}
	else
	{
		return true;
	}
}

bool quadCoderDriver::resetCount(ArduinoInterface& ai)
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
