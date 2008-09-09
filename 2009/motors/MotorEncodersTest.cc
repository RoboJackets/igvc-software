#include <iostream>

#include "MotorEncoders.h"

using namespace std;

int main(void) {
	MotorEncoders encoders;
	MotorEncoders::reply_t status;

	cout << "Size of short = " << sizeof(short) << endl;

	while(true) {
		cout << "Heading = " << encoders.getHeading() << endl;
		//status = encoders.getInfo();
		//cout << "Heading = " << status.heading << endl;
		usleep(100000);
	}
}
