#include "SonarInterface.h"

SonarInterface::SonarInterface(){
	arint.initLink('s');
}

SonarInterface::~SonarInterface(){
	
}

void SonarInterface::pingindiv(int id){
	byte val[2];
	byte msg[2] = {SPING_IND,(byte)id};
	//Send and recieve packets
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	//Sonar actually sends data here (to be retrieved later)...store it.
	val[1] = *(rx_p.data+0);  //<-- data gets switched from arduino to laptop for some reason, find out why
	val[0] = *(rx_p.data+1);
	//printf("::>%d,%d\n",(char)val[0],(char)val[1]);
	memcpy(&(sonar_ranges[id]),&val,2);
	sonar_ranges[id] *= 10; // convert from cm to mm
}

int SonarInterface::readindiv(int id){
	return(sonar_ranges[id]);
}

void SonarInterface::pingall(void){
	byte msg[1] = {SPING_ALL};
	DataPacket rx_p;
	byte val[2];
	int i = 0;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	while(*(rx_p.data+(i*3)+0) != 0xFF){
		val[2] = *(rx_p.data+(i*3)+0);
		val[1] = *(rx_p.data+(i*3)+1);  //<-- data gets switched from arduino to laptop for some reason, find out why
		val[0] = *(rx_p.data+(i*3)+2);

		//printf("::>%d--%d,%d\n",(char)val[2],(char)val[0],(char)val[1]);
		memcpy(&(sonar_ranges[rx_p.data[i*3]]),&val,2);
		sonar_ranges[rx_p.data[i*3]] *= 10; //convert from cm to mm
		i++;
	}

}

void SonarInterface::setAutoModeOn(){ //Not easy to implement --> recieves packet from the ard without sending from laptop
	byte msg[2] = {SET_AUTO_ALL, 1};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}
void SonarInterface::setAutoModeOff(){ //Not easy to implement --> recieves packet from the ard without sending from laptop
	byte msg[2] = {SET_AUTO_ALL, 0};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::setAllGain(int gainval){
	//constrain	
	if(gainval>31) gainval = 31;
	//send and recieve
	byte msg[2] = {SET_GAIN_ALL,(byte)gainval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::setIndivGain(int id, int gainval){
	//constrain	
	if(gainval>31) gainval = 31;
	//send and recieve
	byte msg[3] = {SET_GAIN_IND,(byte)id,(byte)gainval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

int SonarInterface::getIndivGain(int id){
	byte msg[2] = {GET_GAIN_IND,(byte)id};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	//printf("::>%d\n",rx_p.data[0]);
	return((int)rx_p.data[0]);
}

void SonarInterface::setAllMRange(int mrangeval){
	//convert from mm to magic srf08 binary equiv
	byte msg[2] = {SET_MRANGE_ALL,(((mrangeval-43.0)/43.0)+1)};
	printf(":md:>%d\n",(int)((mrangeval-43.0)/43.0));
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void SonarInterface::setIndivMRange(int id, int mrangeval){
	//constrain	
	//send and recieve
	byte msg[3] = {SET_MRANGE_IND,(byte)id,(byte)(((mrangeval-43.0)/43.0)+1)};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

int SonarInterface::getIndivMRange(int id){
	byte msg[2] = {GET_MRANGE_IND,id};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)((((int)rx_p.data[0]+1)*43.0)+43.0));
}
void SonarInterface::setFreq(int freqval){
	byte msg[2] = {SET_FREQ,(byte)freqval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);	
}

void SonarInterface::setStepTotal(int stepnum){
	byte msg[2] = {SET_STEP_TOT,(byte)stepnum};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);	
}

void SonarInterface::setIndivStep(int id, int stepval){ //cannot set step stepvar to total step val
	byte msg[3] = {SET_STEP_IND,(byte)id, (byte)stepval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);	
}

void SonarInterface::setIndivActive(int id, int activeval){
	byte msg[3] = {SET_ACTIVE_IND,(byte)id, (byte)activeval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);	
}

int SonarInterface::getIndivActive(int id){
	byte msg[2] = {GET_ACTIVE_IND,(byte)id};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((bool)rx_p.data[0]);
}

int SonarInterface::getIndivStep(int id){
	byte msg[2] = {GET_STEP_IND,(byte)id};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}

int SonarInterface::getFreq(){
	byte msg[1] = {GET_FREQ};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}	

int SonarInterface::getStepTotal(){
	byte msg[1] = {GET_STEP_TOT};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}
