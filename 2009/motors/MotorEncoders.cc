#include "./ArduinoMotorEncoders/MotorEncoders.h"
#include <sys/time.h>


MotorEncoders::MotorEncoders(void) : ArduinoInterface() {
}


MotorEncoders::reply_t MotorEncoders::getInfo(void) {
	reply_t status;
	getStatus(&status, sizeof(reply_t));

	return(status);
}

deltas MotorEncoders::getDeltas(){
	deltas data;

	if(MotorEncoders::func != SEND_DTICK){
		MotorEncoders::setFunc(SEND_DTICK);
		MotorEncoders::func = SEND_DTICK;
	}
	byte status[6];
	getStatus(status, 6);

	memcpy(&deltas.dl, status, sizeof(deltas.dl));
	memcpy(&deltas.dr, status+sizeof(deltas.dr), sizeof(deltas.dr));
	memcpy(&deltas.time, status+sizeof(deltas.time), sizeof(deltas.time));

	return(data);
}

double MotorEncoders::getHeading(void) {
	deltas data = MotorEncoders::getDeltas();
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
	deltas data = MotorEncoders::getDeltas();

	double dt = deltas.dt / COUNTER_RATE

	double wr = deltas.dr / dt;
	double wl = deltas.dl / dt;
	
	double w = (wr - wl)*WHEEL_RADIUS/WHEEL_BASE

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
