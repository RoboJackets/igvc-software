#include "./ArduinoMotorEncoders/MotorEncoders.h"

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

double getRotVel(void){
	deltas data = MotorEncoders::getDeltas();

	double dt = deltas.dt / COUNTER_RATE

	double wr = deltas.dr / dt;
	double wl = deltas.dl / dt;
	
	double w = (wr - wl)*WHEEL_RADIUS/WHEEL_BASE

	return(w);
}

bool MotorEncoders::setHeading(double heading) {
	MotorEncoders::heading = heading;
//	setVar(HEADING, &heading, sizeof(double));
}

bool MotorEncoders::setFunc(int mode){
	setVar(RET_T, &mode, sizeof(int));
}

bool MotorEncoders::setSendMode(int mode){
	setVar(PUSHPULL, &mode, sizeof(int));
}
