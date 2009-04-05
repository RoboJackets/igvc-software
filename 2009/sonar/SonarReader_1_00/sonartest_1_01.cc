//SONAR_TEST
//notes:had to disable setArduinoTime() within initLink in 'ArduinoInterface.cc'; caused serious problems. << FIXED
//need to add constraints for sending data
//SonarDevice IDs -- 0 THRU 15
//make arduino side faster - dont declare variables that are useless
	

#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <stdio.h>
#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

#define ARDUINO_SD_CMD 's'

//using namespace std;

enum sd_optype_t{	SREAD_ALL,SPING_ALL,SREAD_IND,SPING_IND,SET_GAIN_ALL,SET_GAIN_IND,SET_MRANGE_ALL,SET_MRANGE_IND,
					SET_FREQ,SET_STEP_IND,SET_STEP_TOT,SET_ACTIVE_IND,SET_ECHONUM,GET_GAIN_IND,GET_MRANGE_IND,
					GET_ACTIVE_IND,GET_STEP_IND,GET_FREQ,GET_STEP_TOT,START_SONDEV, SET_AUTO_ALL};
short int sonar_ranges[16];
//short int rdat;

ArduinoInterface arint;

void pingindiv(int id){
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

int readindiv(int id){
	return(sonar_ranges[id]);
}

void pingall(void){
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

void setAutoModeOn(){ //Not easy to implement --> recieves packet from the ard without sending from laptop
	byte msg[2] = {SET_AUTO_ALL, 1};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}
void setAutoModeOff(){ //Not easy to implement --> recieves packet from the ard without sending from laptop
	byte msg[2] = {SET_AUTO_ALL, 0};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void setAllGain(int gainval){
	//constrain	
	if(gainval>31) gainval = 31;
	//send and recieve
	byte msg[2] = {SET_GAIN_ALL,(byte)gainval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void setIndivGain(int id, int gainval){
	//constrain	
	if(gainval>31) gainval = 31;
	//send and recieve
	byte msg[3] = {SET_GAIN_IND,(byte)id,(byte)gainval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

int getIndivGain(int id){
	byte msg[2] = {GET_GAIN_IND,(byte)id};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	//printf("::>%d\n",rx_p.data[0]);
	return((int)rx_p.data[0]);
}

void setAllMRange(int mrangeval){
	//convert from mm to magic srf08 binary equiv
	byte msg[2] = {SET_MRANGE_ALL,(((mrangeval-43.0)/43.0)+1)};
	printf(":md:>%d\n",(int)((mrangeval-43.0)/43.0));
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

void setIndivMRange(int id, int mrangeval){
	//constrain	
	//send and recieve
	byte msg[3] = {SET_MRANGE_IND,(byte)id,(byte)(((mrangeval-43.0)/43.0)+1)};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
}

int getIndivMRange(int id){
	byte msg[2] = {GET_MRANGE_IND,id};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)((((int)rx_p.data[0]+1)*43.0)+43.0));
}
void setFreq(int freqval){
	byte msg[2] = {SET_FREQ,(byte)freqval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);	
}

void setStepTotal(int stepnum){
	byte msg[2] = {SET_STEP_TOT,(byte)stepnum};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);	
}

void setIndivStep(int id, int stepval){ //cannot set step stepvar to total step val
	byte msg[3] = {SET_STEP_IND,(byte)id, (byte)stepval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);	
}

void setIndivActive(int id, int activeval){
	byte msg[3] = {SET_ACTIVE_IND,(byte)id, (byte)activeval};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,3);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);	
}

int getIndivActive(int id){
	byte msg[2] = {GET_ACTIVE_IND,(byte)id};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((bool)rx_p.data[0]);
}

int getIndivStep(int id){
	byte msg[2] = {GET_STEP_IND,(byte)id};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,2);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}

int getFreq(){
	byte msg[1] = {GET_FREQ};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}	

int getStepTotal(){
	byte msg[1] = {GET_STEP_TOT};
	DataPacket rx_p;
	arint.sendCommand(ARDUINO_SD_CMD,msg,1);
	arint.recvCommand(rx_p.header.cmd, rx_p.data);
	return((int)rx_p.data[0]);
}	


int main(void){
	int data[10];
	int dat;	
	printf("\nInitializing Link...");
	arint.initLink('s');
	setFreq(14);

	setStepTotal(4);
	setIndivStep(0,0); setIndivActive(0,0);
	setIndivStep(1,0); setIndivActive(1,1);
	setIndivStep(2,0); setIndivActive(2,1);
	setIndivStep(3,1); setIndivActive(3,1);
	setIndivStep(4,1); setIndivActive(4,1);
	setIndivStep(5,2); setIndivActive(5,1);
	setIndivStep(6,2); setIndivActive(6,1);
	setIndivStep(7,2); setIndivActive(7,1);
	setIndivStep(8,3); setIndivActive(8,1);
	setIndivStep(9,3); setIndivActive(9,1);

	pingindiv(0);
	

	setIndivGain(0,15);
	pingall();
	for(int i=0; i<10; i++){
		dat = readindiv(i);
		printf(":s:>%d--%d\n",i,dat);
	}
	dat = getIndivGain(0);
	printf(":s:>%dg--%d\n",0,dat);
	setAllGain(30);
	setIndivMRange(0,1500);
	pingall();
	for(int i=0; i<10; i++){
		dat = readindiv(i);
		printf(":s:>%d--%d\n",i,dat);
	}
	dat = getIndivGain(0);
	printf(":s:>%dg--%d\n",0,dat);
	dat = getIndivMRange(0);
	printf(":mr:>%dm--%d\n",0,dat);
	dat = getIndivActive(0);
	printf(":a0:>%d\n",dat);
	dat = getIndivActive(1);
	printf(":a1:>%d\n",dat);
	dat = getIndivStep(0);
	printf(":s0:>%d\n",dat);
	dat = getIndivStep(5);
	printf(":s1:>%d\n",dat);
	dat = getFreq();
	printf(":f:>%d\n",dat);
	dat = getStepTotal();
	printf(":st:>%d\n",dat);
	
	printf("Success!\nProgram Terminated.\n");
	return 0;
}


