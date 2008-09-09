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

double MotorEncoders::getHeading(void) {
	reply_t status;
	getStatus(&status, sizeof(reply_t));

	return(status.heading);
}

bool MotorEncoders::setHeading(double heading) {
	setVar(HEADING, &heading, sizeof(double));
}

