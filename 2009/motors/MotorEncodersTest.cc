#include <iostream>
#include <cstdio>
#include "MotorEncoders.h"

using namespace std;

int main(void) {
	MotorEncoders encoders;
	//MotorEncoders::reply_t status;

	cout << "Size of short = " << sizeof(short) << endl;
	cout << "size of struct = " << sizeof(reply_t) << endl;

	int packetnum = 1;

	while(true) {
		//cout << "Heading = " << encoders.getHeading() << endl;
		//encoders.setArduinoClock();
		reply_t packet = encoders.getInfo();
		printf("timestamp (s): %d\n", (packet.timestamp) / 1000);
		printf("packetnum: %d\n", packet.packetnum);

		if(packetnum != packet.packetnum){
			cout << "dropped packet" << endl;
			exit(0);
		}
		packetnum++;

		printf("dl: %X\n", (unsigned short)packet.dl);
		printf("dr: %X\n", (unsigned short)packet.dr);
		printf("dt: %X\n", (unsigned short)packet.dt);
		cout << "\n\n";
		//status = encoders.getInfo();
		//cout << "Heading = " << status.heading << endl;
		usleep(.01*1e6);
	}
}
