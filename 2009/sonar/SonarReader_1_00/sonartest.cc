//SONAR_TEST
//notes:had to disable setArduinoTime() within initLink in 'ArduinoInterface.cc'; caused serious problems.


#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <stdio.h>
#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

#define ARDUINO_SD_CMD 's'

using namespace std;

enum sd_optype_t {	SREAD_ALL,SPING_ALL,SREAD_IND,SPING_IND,SET_GAIN_ALL,SET_GAIN_IND,SET_MRANGE_ALL,SET_MRANGE_IND,
				   SET_FREQ,SET_STEP_IND,SET_STEP_TOT,SET_ACTIVE_IND,SET_ECHONUM,GET_GAIN_IND,GET_MRANGE_IND,
				   GET_ACTIVE_IND,GET_STEP_IND,GET_FREQ,GET_STEP_TOT,START_SONDEV, SET_AUTO_ALL
				 };
int sonar_ranges[16];

ArduinoInterface arint;

void pingindiv(int id)
{

	byte msg[2] = {SPING_IND,(byte)id};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);

}

int readindiv(int id)
{

	byte msg[2],val[2];
	int ret;
	msg[0] = SREAD_IND;
	msg[1] = (byte)id;

	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd,rx_p.data);
	val[0] = *(rx_p.data+0);
	val[1] = *(rx_p.data+1);
	memcpy(&ret,&val,2);
	return(ret);
}

void pingall(void)
{
	byte msg[1] = {SPING_ALL};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void setAutoModeOn()
{
	byte msg[2] = {SET_AUTO_ALL, 1};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}
void setAutoModeOff()
{
	byte msg[2] = {SET_AUTO_ALL, 0};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}


int main(void)
{
	int data[10];
	int dat;
	printf("\nInitializing Link...");
	arint.initLink('s');
	pingindiv(0);
	dat = readindiv(0);
	printf("%d\n",dat);


	printf("Success!\nProgram Terminated.\n");
	return 0;
}


