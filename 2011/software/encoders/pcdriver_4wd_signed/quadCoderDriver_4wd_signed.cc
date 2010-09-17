
#include "quadCoderDriver_4wd_signed.hpp"

//false on success

quadCoderDriver_4wd_signed::quadCoderDriver_4wd_signed() : fr(ENCODER_IF_FOR_RIGHT_BOARD), fl(ENCODER_IF_FOR_LEFT_BOARD), br(ENCODER_IF_AFT_RIGHT_BOARD), bl(ENCODER_IF_AFT_LEFT_BOARD)
{
	m_connected = true;
}

bool quadCoderDriver_4wd_signed::getEncoderVel(double& rvelFWD, double& lvelFWD,double& rvelAFT, double& lvelAFT)
{
	bool ret = false;
	ret |= fr.getEncoderVel(rvelFWD);
	ret |= fl.getEncoderVel(lvelFWD);
	ret |= br.getEncoderVel(rvelAFT);
	ret |= bl.getEncoderVel(lvelAFT);
	return ret;
}

bool quadCoderDriver_4wd_signed::resetCount()
{
	bool ret = false;

	ret |= fr.resetCount();
	ret |= fl.resetCount();
	ret |= br.resetCount();
	ret |= bl.resetCount();

	return ret;
}
