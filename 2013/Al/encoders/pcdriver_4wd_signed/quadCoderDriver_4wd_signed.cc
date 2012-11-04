
#include "quadCoderDriver_4wd_signed.hpp"

//false on success

quadCoderDriver_4wd_signed::quadCoderDriver_4wd_signed() : front(ENCODER_IF_FOR_BOARD), back(ENCODER_IF_AFT_BOARD)
{
	m_connected = true;
}

bool quadCoderDriver_4wd_signed::getEncoderVel(double& rvelFWD, double& lvelFWD,double& rvelAFT, double& lvelAFT)
{
	bool ret = false;
	ret |= front.getEncoderVel(lvelFWD, rvelFWD);
	ret |= back.getEncoderVel(lvelAFT, rvelAFT);
	return ret;
}

bool quadCoderDriver_4wd_signed::resetCount()
{
	bool ret = false;

	ret |= front.resetCount();
	ret |= back.resetCount();

	return ret;
}
