#include <iostream>
#include <cstdio>
#include <cstring>
#include "MotorEncoders.h"
#include "../arduino/DataPacket.hpp"
using namespace std;

int main(void) {
	MotorEncoders encoders;
	//MotorEncoders::reply_t status;

	cout << "Size of short = " << sizeof(short) << endl;
	cout << "Size of header = " << sizeof(DataPacket::header_t) << endl;
	cout << "size of struct = " << (sizeof(DataPacket::encoder_reply_t)) << endl;

	//int exp_packetnum = 1;
	int i;
	for( i = 0; i < 10; i++){
		//cout << "Heading = " << encoders.getHeading() << endl;
		//encoders.setArduinoClock();
		//reply_t packet = encoders.getInfo();

		DataPacket packet;

		//((ArduinoInterface)encoders).setArduinoClock();

		encoders.getInfo_class(&packet);

		DataPacket::encoder_reply_t parsed_data;
		memcpy(&parsed_data, packet.data, sizeof(DataPacket::encoder_reply_t));

		//DataPacket::encoder_reply_t data = encoders.getInfo();
		//printf("timestamp (s): %d\n", (packet.packet->timestamp) / 1000);
		//printf("packetnum: %d\n", packet.packet->packetnum);

		//printf("dl: %X\n", (unsigned short)parsed_data.dl);
		//printf("dr: %X\n", (unsigned short)parsed_data.dr);
		//printf("dt: %X\n", (unsigned)parsed_data.dt);
		cout << parsed_data << "\n\n";

		//for(int i = 0; i < sizeof(DataPacket::header_t); i++){
		//	printf("%X",  ((byte*)(&packet.header))[i]);
		//}
		for(int i = 0; i < sizeof(DataPacket::encoder_reply_t); i++){
			//printf("%X",  ((byte*)(&parsed_data))[i]);
			printf("%X", packet.data[i] );
		}
		cout << "\n\n";
		//status = encoders.getInfo();
		//cout << "Heading = " << status.heading << endl;
		usleep(.005*1e6);
	}

	DataPacket pk;
	((ArduinoInterface)encoders).arduinoResendPacket(i, &pk);

	DataPacket::encoder_reply_t parsed_data;
	if (pk.data != NULL){
		memcpy(&parsed_data, pk.data, sizeof(DataPacket::encoder_reply_t));
	}
	//printf("timestamp (s): %d\n", (parseddata->timestamp) / 1000);
	//printf("packetnum: %d\n", parseddata->packetnum);

	printf("dl: %X\n", (unsigned short)parsed_data.dl);
	printf("dr: %X\n", (unsigned short)parsed_data.dr);
	printf("dt: %X\n", (unsigned)parsed_data.dt);

	//delete[] pk;
}
