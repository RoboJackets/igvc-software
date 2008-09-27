#include "MotorEncoders.h"

MotorEncoders::MotorEncoders(void) : ArduinoInterface() {
}


MotorEncoders::reply_t MotorEncoders::getInfo(void) {
	reply_t status;
	getStatus(&status, sizeof(reply_t));

	return(status);
}

/*int MotorEncoders::getLeftTick(void) {
}

int MotorEncoders::getRightTick(void) {
}*/

int getDeltaTick(){

}

int getDeltaTime(){

}

deltas MotorEncoders::getDeltas(){
	deltas data;

	if(MotorEncoders::ret_mode != ALL){
		MotorEncoders::setRetMode(ALL);
		MotorEncoders::ret_mode = ALL;
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

	double dt = (current.time - previous.time) / COUNTER_RATE
//	int dl = delta(current.leftMotorTick, previous.leftMotorTick, 100, TOTAL_ENCODER_TICKS - 1,  LEFT_MOTOR_ENCODER_DIRECTION); 
//	int dr = delta(current.rightMotorTick, previous.rightMotorTick, 100, TOTAL_ENCODER_TICKS - 1, RIGHT_MOTOR_ENCODER_DIRECTION);

	double wr = deltas.dr / deltas.dt;
	double wl = deltas.dl / deltas.dt;
	
	double w = (wr - wl)*WHEEL_RADIUS/WHEEL_BASE

	return(w);
}

bool MotorEncoders::setHeading(double heading) {
	MotorEncoders::heading = heading;
//	setVar(HEADING, &heading, sizeof(double));
}

bool MotorEncoders::setRetMode(int mode){
	setVar(RET_T, &mode, sizeof(int));
}

bool MotorEncoders::setSendMode(int mode){
	setVar(PUSHPULL, &mode, sizeof(int));
}
