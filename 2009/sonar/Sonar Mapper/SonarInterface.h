#ifndef SONAR_INTERFACE_H
#define SONAR_INTERFACE_H

#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <stdio.h>
#include "ArduinoInterface.h"
#include "ArduinoCmds.hpp"

#define ARDUINO_SD_CMD 's'

class SonarInterface{

	public:
	SonarInterface(void);
	virtual ~SonarInterface(void);

	void initSonDev(int num, int gain, int mrange, int freq);

	void pingIndiv(int id);
	void pingAll(void);
	int readIndiv(int id);
	int readIndiv(int id, char type);
	int updateAll(void);
	void setAutopilot(int on_off);

	void setAllGain(int gainval);
	void setIndivGain(int id, int gainval);
	void setAllMRange(int mrangeval);
	void setIndivMRange(int id, int mrangeval);
	void setFreq(int freqval);
	void setStepTotal(int stepnum);
	void setIndivStep(int id, int stepval);
	void setIndivActive(int id, int activeval);

	int getIndivMRange(int id);
	int getIndivGain(int id);
	int getIndivActive(int id);
	int getIndivStep(int id);
	int getFreq(void);	
	int getStepTotal(void);

	private:
	enum sd_optype_t{SREAD_ALL,SPING_ALL,SREAD_IND,SPING_IND,SET_GAIN_ALL,SET_GAIN_IND,SET_MRANGE_ALL,SET_MRANGE_IND,
			SET_FREQ,SET_STEP_IND,SET_STEP_TOT,SET_ACTIVE_IND,SET_ECHONUM,GET_GAIN_IND,GET_MRANGE_IND,
			GET_ACTIVE_IND,GET_STEP_IND,GET_FREQ,GET_STEP_TOT,START_SONDEV, SET_AUTO_ALL, SDEV_ADDRSET};
	short int sonar_ranges[16];
	ArduinoInterface arint;
	int ConvertID(int id);
};


#endif
