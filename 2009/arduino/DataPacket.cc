#include "DataPacket.h"

PCdatapacket::PCdatapacket(){
	data = NULL;
}

PCdatapacket::~PCdatapacket(){
	if(data != NULL){
		delete[] data;
	}
}

EncoderData::EncoderData(){
	data = new byte[sizeof(reply_t)];
	packet = (reply_t *) data;

}

/*
EncoderData::~EncoderData(){
	packet = NULL;
}
*/
