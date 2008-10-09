#include "DataPacket.h"

PCdatapacket::PCdatapacket(){
	data = NULL;
	len = 0;
}

PCdatapacket::~PCdatapacket(){
	if(data != NULL){
		delete[] data;
	}
}

EncoderData::EncoderData(){
	len = sizeof(reply_t)
	data = new byte[sizeof(reply_t)];
	packet = (reply_t *) data;
}

CurrentData::CurrentData(){
	len = sizeof(reply_t)
	data = new byte[sizeof(reply_t)];
	packet = (reply_t *) data;
}

/*
EncoderData::~EncoderData(){
	packet = NULL;
}
*/
