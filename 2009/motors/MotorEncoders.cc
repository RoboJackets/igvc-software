#include "./ArduinoMotorEncoders/MotorEncoders.h"
#include <sys/time.h>
#include <string>

MotorEncoders::MotorEncoders(void) : ArduinoInterface() {

	unsigned int rx_packetnum = 1;
	unsigned int tx_packetnum = 1;

}


MotorEncoders::reply_t MotorEncoders::getInfo(void) {
	reply_t status;
	getStatus(&status, sizeof(reply_t));

	return(status);
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
	reply_t data = MotorEncoders::getInfo();
	MotorEncoders::heading += ( ((double)(data.dr - data.dl)) * (double)RAD_PER_ENCODER_TICK * (double)WHEEL_RADIUS / (double)WHEEL_BASE );
	while (MotorEncoders::heading >= TWO_PI) { // this could be done more effiecently
		MotorEncoders::heading -= TWO_PI;
	}

	while (heading < 0) {
		MotorEncoders::heading += TWO_PI;
	}


	return(MotorEncoders::heading);
}

double MotorEncoders::getRotVel(void){
	//EncoderPacket data = MotorEncoders::getDeltas();
	reply_t data = MotorEncoders::getInfo();

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

	double dt = data.dt / COUNTER_RATE

	double wr = data.dr / dt;
	double wl = data.dl / dt;
	
	double w = (wr - wl)*WHEEL_RADIUS/WHEEL_BASE

	return(w);
}

void MotorEncoders::setHeading(double heading) {
	this.heading = heading;
//	setVar(HEADING, &heading, sizeof(double));
}

bool MotorEncoders::setFunc(int mode){
	return( setVar(RET_T, &mode, sizeof(int)) );
}

bool MotorEncoders::setSendMode(int mode){
	return( setVar(PUSHPULL, &mode, sizeof(int)) );
}

bool MotorEncoders::setLogFile(string str){
	this.logfile = str;
}

bool MotorEncoders::setArduinoClock(){
	//int time = 
	struct timeval time;
	struct timezone tz;
	gettimeofday(&time, &tz);

	byte[4] timedata;

	long int millis = time.tv_sec*1000 + ((long int)((double)tv_usec/1000));

	memcpy(timedata+4, millis, 4);

	return( setVar(SETCLK1, timedata, 4 ) );
}
bool MotorEncoders::setSendMode(int mode, int int_rt){
	bool a = setVar(PUSHPULL, &mode, sizeof(int));
	bool b = setVar(INTEROG_DL, &int_rt, sizeof(int);

	return(a && b);
}
