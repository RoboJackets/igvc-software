#include "MotorEncoders.h"
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <sys/timex.h>
#include <string.h>
#include <cmath>

typedef unsigned char byte;

MotorEncoders::MotorEncoders(void) : ArduinoInterface() {

	unsigned int rx_packetnum = 1;
	unsigned int tx_packetnum = 1;

}


EncoderData::reply_t MotorEncoders::getInfo(void) {
	EncoderData::reply_t status;
	
	getStatus(&status, sizeof(status));

	return(status);
}

bool MotorEncoders::getInfo_class(EncoderData * status) {
	//EncoderData status;
	//byte * data = new byte[sizeof(EncoderData::reply_t)];
	bool ret = getStatus(status->data, sizeof(EncoderData::reply_t));
	//status.setDataPointer(data);
	//status.packnum = rx_packetnum;
	//rx_packetnum++;
	//return(status);
	return(ret);
}

/*
EncoderPacket MotorEncoders::getDeltas(){
	EncoderPacket data;

	long timestamp;
	long packetnum;

	if(MotorEncoders::func != SEND_DTICK){
		MotorEncoders::setFunc(SEND_DTICK);
		MotorEncoders::func = SEND_DTICK;
	}
	byte status[14];
	getStatus(status, 14);

	memcpy(&data.timestamp, status, 4)
	memcpy(&data.packetnum, status+4, 4)
	memcpy(&data.dl, status+10, 2);
	memcpy(&data.dr, status+12, 2);
	memcpy(&data.time, status+14, 2);

	return(data);
}
*/

double MotorEncoders::getHeading(void) {
	//EncoderPacket data = MotorEncoders::getDeltas();
	EncoderData::reply_t data = MotorEncoders::getInfo();
	MotorEncoders::heading += ( ((double)(data.dr - data.dl)) * (double)RAD_PER_ENCODER_TICK * (double)WHEEL_RADIUS / (double)WHEEL_BASE );
	while (MotorEncoders::heading >= (2*M_PI)) { // this could be done more effiecently
		MotorEncoders::heading -= (2*M_PI);
	}

	while (heading < 0) {
		MotorEncoders::heading += (2*M_PI);
	}


	return(MotorEncoders::heading);
}

double MotorEncoders::getRotVel(void){
	//EncoderPacket data = MotorEncoders::getDeltas();
	EncoderData::reply_t data = MotorEncoders::getInfo();
/*
	if(data.packetnum != rx_packetnum){
		int trynum = 0;
		do{		
			resendPacket(rx_packetnum, &data, sizeof(data));
			if(data.packetnum == rx_packetnum){
				rx_packetnum++;
				break;
			}
			trynum++;
		}while((data.packetnum != rx_packetnum) && (trynum < 5))
		return(NaN);
	}
	else{
		rx_packetnum++;
	}
*/
	double dt = data.dt / COUNTER_RATE;

	double wr = data.dr / dt;
	double wl = data.dl / dt;
	
	double w = (wr - wl)*WHEEL_RADIUS/WHEEL_BASE;

	return(w);
}

void MotorEncoders::setHeading(double heading) {
	MotorEncoders::heading = heading;
//	setVar(HEADING, &heading, sizeof(double));
}

bool MotorEncoders::setFunc(int mode){
	return( setVar(RET_T, &mode, sizeof(int)) );
}

bool MotorEncoders::setSendMode(int mode){
	return( setVar(PUSHPULL, &mode, sizeof(int)) );
}

bool MotorEncoders::setLogFile(string str){
	logfile = str;
}

bool MotorEncoders::setArduinoClock(){
	//int time = 
	struct timeval time;
	struct timezone tz;
	gettimeofday(&time, &tz);

	unsigned char timedata[4];

	long int millis = time.tv_sec*1000 + ((long int)((double)time.tv_usec/1000));

	//memcpy(timedata, &millis, 4);

	return( setVar(SETCLK, &millis, 4 ) );
}
bool MotorEncoders::setSendMode(int mode, int int_rt){
	bool a = setVar(PUSHPULL, &mode, sizeof(int));
	bool b = setVar(INTEROG_DL, &int_rt, sizeof(int));

	return(a && b);
}


