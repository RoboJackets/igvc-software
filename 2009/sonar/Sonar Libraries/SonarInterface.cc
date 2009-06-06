#include "SonarInterface.h"

SonarInterface::SonarInterface()
{
	arint.initLink('s');
}

SonarInterface::~SonarInterface()
{

}

void SonarInterface::initSonDev(int num, int gain, int mrange, int freq)
{
	byte msg[5] = {START_SONDEV,(byte)num,(byte)gain,(byte)(((mrange-43.0)/43.0)+1),(byte)freq};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,5);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::pingIndiv(int id)
{
	id = ConvertID(id);
	byte msg[2] = {SPING_IND,(byte)id};
	//Send and recieve packets
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

int SonarInterface::readIndiv(int id)
{
	byte val[2];
	byte msg[2] = {SREAD_IND,(byte)id};
	id = ConvertID(id);
	//Send and recieve packets
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	//Recieve Range Data Now; Store for Later
	val[1] = *(rx_p.data+0);  //<-- data gets switched during transfer from arduino to laptop...?
	val[0] = *(rx_p.data+1);
	if (val[0] == 0xFF && val[1] == 0xFF)
	{
		return(-1);
	}
	else
	{
		memcpy(&(sonar_ranges[id]),&val,2);
		sonar_ranges[id] *= 10; // convert from cm to mm
		return(sonar_ranges[id]);
	}
}

int SonarInterface::readIndiv(int id, char type)
{
	id = ConvertID(id);
	if (tolower(type) == 'o')
	{
		return(sonar_ranges[id]);
	}
	else if (tolower(type) == 'n')
	{
		return(readIndiv(id));
	}
	else
	{
		return(-2);
	}
}

void SonarInterface::pingAll(void)
{
	byte msg[1] = {SPING_ALL};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

int SonarInterface::updateAll(void)
{
	byte msg[1] = {SREAD_ALL};
	DataPacket rx_p;
	byte val[2];
	int i = 0;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	if (*(rx_p.data) == 0xFF) return(-1);
	while (*(rx_p.data+(i*3)+0) != 0xFF)
	{
		val[2] = *(rx_p.data+(i*3)+0);
		val[1] = *(rx_p.data+(i*3)+1);  //<-- nybble gets flipped from arduino to laptop for some reason, find out why
		val[0] = *(rx_p.data+(i*3)+2);
		memcpy(&(sonar_ranges[rx_p.data[i*3]]),&val,2);
		sonar_ranges[rx_p.data[i*3]] *= 10; //convert from cm to mm
		i++;
	}
	return(0);
}

void SonarInterface::setAutopilot(int on_off)
{
	byte msg[2] = {SET_AUTO_ALL, (byte)on_off};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::setAllGain(int gainval)
{
	//constrain
	if (gainval>31) gainval = 31;
	//send and recieve
	byte msg[2] = {SET_GAIN_ALL,(byte)gainval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::setIndivGain(int id, int gainval)
{
	//constrain
	if (gainval>31) gainval = 31;
	id = ConvertID(id);
	//send and recieve
	byte msg[3] = {SET_GAIN_IND,(byte)id,(byte)gainval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

int SonarInterface::getIndivGain(int id)
{
	byte msg[2] = {GET_GAIN_IND,(byte)id};
	id = ConvertID(id);
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}

void SonarInterface::setAllMRange(int mrangeval)
{
	//convert from mm to magic srf08 binary equiv
	byte msg[2] = {SET_MRANGE_ALL,(((mrangeval-43.0)/43.0)+1)};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::setIndivMRange(int id, int mrangeval)
{
	//constrain
	id = ConvertID(id);
	//send and recieve
	byte msg[3] = {SET_MRANGE_IND,(byte)id,(byte)(((mrangeval-43.0)/43.0)+1)};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

int SonarInterface::getIndivMRange(int id)
{
	byte msg[2] = {GET_MRANGE_IND,id};
	id = ConvertID(id);
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)((((int)rx_p.data[0]+1)*43.0)+43.0));
}
void SonarInterface::setFreq(int freqval)
{
	byte msg[2] = {SET_FREQ,(byte)freqval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::setStepTotal(int stepnum)
{
	byte msg[2] = {SET_STEP_TOT,(byte)stepnum};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::setIndivStep(int id, int stepval)
{
	byte msg[3] = {SET_STEP_IND,(byte)id, (byte)stepval};
	id = ConvertID(id);
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::setIndivActive(int id, int activeval)
{
	byte msg[3] = {SET_ACTIVE_IND,(byte)id, (byte)activeval};
	id = ConvertID(id);
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

int SonarInterface::getIndivActive(int id)
{
	byte msg[2] = {GET_ACTIVE_IND,(byte)id};
	id = ConvertID(id);
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((bool)rx_p.data[0]);
}

int SonarInterface::getIndivStep(int id)
{
	byte msg[2] = {GET_STEP_IND,(byte)id};
	id = ConvertID(id);
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}

int SonarInterface::getFreq()
{
	byte msg[1] = {GET_FREQ};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}

int SonarInterface::getStepTotal()
{
	byte msg[1] = {GET_STEP_TOT};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}

int SonarInterface::ConvertID(int id)
{
	if (id >= 0xE0)
	{
		return((id/2)-112);
	}
	else
	{
		return(id);
	}
}
