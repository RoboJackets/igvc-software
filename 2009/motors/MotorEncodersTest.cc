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
	for( i = 0; i < 100000; i++){
		//cout << "Heading = " << encoders.getHeading() << endl;

		//DataPacket packet;

		//encoders.getInfo_class(&packet);

		encoder_reply_t parsed_data;
		if(!encoders.getInfo(parsed_data))
		{
			cout << parsed_data << "\n\n";

			cout << "heading: " << encoders.getHeading() << endl;
		}
		else
		{
			cout << "could not get data" << endl;
		}
		//status = encoders.getInfo();
		//cout << "Heading = " << status.heading << endl;
		usleep(.5*1e6);
	}

	DataPacket pk;

	if(!encoders.arduinoInterface.arduinoResendPacket(98, pk))
	{
		cout << pk << endl;

		std::cout << "resend pk debug extern" << std::endl;
		std::cout << "data size: " << (int) pk.header.size << std::endl;
		std::cout << "dataloc: " << (int) pk.data << std::endl;

		if (pk.data != NULL){
			if(pk.header.cmd == 'r'){
				encoder_reply_t parsed_data;
				memcpy(&parsed_data, pk.data, sizeof(encoder_reply_t));
				cout << parsed_data << "\n\n";
			}
		}
	
	}
}
