#include <iostream>
#include <cstdio>
#include "MotorEncoders.h"

using namespace std;

int main(void) {
	MotorEncoders encoders;
	//MotorEncoders::reply_t status;

	cout << "Size of short = " << sizeof(short) << endl;
	cout << "size of struct = " << sizeof(reply_t) << endl;
	while(true) {
		//cout << "Heading = " << encoders.getHeading() << endl;
		reply_t packet = encoders.getInfo();
		printf("timestamp: %X\n", packet.timestamp);
		printf("packetnum: %X\n", packet.packetnum);
		printf("dl: %X\n", packet.dl);
		printf("dr: %X\n", packet.dr);
		printf("dt: %X\n", packet.dt);
		cout << "\n\n";
		//status = encoders.getInfo();
		//cout << "Heading = " << status.heading << endl;
		usleep(300000);
	}
}
