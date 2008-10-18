#include <iostream>
#include <cstdio>
#include "MotorEncoders.h"

using namespace std;

int main(void) {
	MotorEncoders encoders;
	//MotorEncoders::reply_t status;

	cout << "Size of short = " << sizeof(short) << endl;
	cout << "size of struct = " << (sizeof(EncoderData::reply_t)) << endl;

	//int exp_packetnum = 1;
	for(int i = 0; i < 10; i++){
		//cout << "Heading = " << encoders.getHeading() << endl;
		//encoders.setArduinoClock();
		//reply_t packet = encoders.getInfo();
		EncoderData packet;
		encoders.getInfo_class(&packet);
		printf("timestamp (s): %d\n", (packet.packet->timestamp) / 1000);
		printf("packetnum: %d\n", packet.packet->packetnum);
/*
		if(exp_packetnum != packet.packet->packetnum){
			cout << "dropped packet" << endl;
			//exit(0);
			return -1;
		}
		exp_packetnum++;
*/
		printf("dl: %X\n", (unsigned short)packet.packet->dl);
		printf("dr: %X\n", (unsigned short)packet.packet->dr);
		printf("dt: %X\n", (unsigned short)packet.packet->dt);
		cout << "\n\n";
		//status = encoders.getInfo();
		//cout << "Heading = " << status.heading << endl;
		usleep(.005*1e6);
	}
/*
	PCdatapacket pk;
	((ArduinoInterface)encoders).requestPacket(8, &pk, sizeof(EncoderData::reply_t));
	EncoderData::reply_t * parseddata = (EncoderData::reply_t *) pk.data;
	printf("timestamp (s): %d\n", (parseddata->timestamp) / 1000);
	printf("packetnum: %d\n", parseddata->packetnum);

	printf("dl: %X\n", (unsigned short)parseddata->dl);
	printf("dr: %X\n", (unsigned short)parseddata->dr);
	printf("dt: %X\n", (unsigned short)parseddata->dt);
*/
}
