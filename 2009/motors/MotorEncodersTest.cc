#include <iostream>
#include <cstdio>
#include <cstring>
#include "MotorEncoders.h"
#include "../arduino/DataPacket.hpp"
using namespace std;

int main(void) {
	MotorEncoders encoders;

//	cout << "Size of short = " << sizeof(short) << endl;
//	cout << "Size of header = " << sizeof(DataPacket::header_t) << endl;
//	cout << "size of struct = " << (sizeof(DataPacket::encoder_reply_t)) << endl;

	//int exp_packetnum = 1;
	int i;
	for( i = 0; i < 10000; i++){
		//cout << "Heading = " << encoders.getHeading() << endl;

		DataPacket packet;

		encoders.getInfo_class(&packet);

		DataPacket::encoder_reply_t parsed_data;
		memcpy(&parsed_data, packet.data, sizeof(DataPacket::encoder_reply_t));

		cout << packet;
		cout << parsed_data << "\n\n";

		//for(int i = 0; i < sizeof(DataPacket::header_t); i++){
		//	printf("%X",  ((byte*)(&packet.header))[i]);
		//}
//		for(int i = 0; i < sizeof(DataPacket::encoder_reply_t); i++){
			//printf("%X",  ((byte*)(&parsed_data))[i]);
			//printf("%X", packet.data[i] );
//		}
		cout << "\n\n";
		//status = encoders.getInfo();
		//cout << "Heading = " << status.heading << endl;
		usleep(.001*1e6);
	}

	DataPacket * pk = new DataPacket;
	encoders.arduinoInterface.arduinoResendPacket(40070, pk);

	cout << *pk << endl;

	std::cout << "resend pk debug extern" << std::endl;
	std::cout << "data size: " << (int) pk->header.size << std::endl;
	std::cout << "dataloc: " << (int) pk->data << std::endl;

	if (pk->data != NULL){
		if(pk->header.cmd == 'r'){
			DataPacket::encoder_reply_t parsed_data;
			memcpy(&parsed_data, pk->data, sizeof(DataPacket::encoder_reply_t));
			cout << parsed_data << "\n\n";
		}
	}
	delete pk;



	//delete[] pk;
}
