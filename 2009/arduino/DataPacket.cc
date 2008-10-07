#include "DataPacket.h"

PCdatapacket::~PCdatapacket(){
	delete[] data;
}

void EncoderData::setDataPointer(void * ptr){
	packet = (reply_t *)ptr;
	data = (byte *)ptr;
}

