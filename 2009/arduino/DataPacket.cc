#include "DataPacket.h"

PCdatapacket::~PCdatapacket(){
	delete[] data;
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
