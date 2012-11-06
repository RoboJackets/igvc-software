#include "current_sensors.hpp"

int getLeftCurrentADCVal()
{
	int cl = analogRead(3);
	return cl;
}

int getRightCurrentADCVal()
{
	int cr = analogRead(4);
	return cr;
}

current_reply_t getBothCurrentADCVal()
{
	current_reply_t pk;
	pk.il = getLeftCurrentADCVal();
	pk.ir = getRightCurrentADCVal();
	return pk;
}
