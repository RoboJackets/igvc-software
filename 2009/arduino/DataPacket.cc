#include "DataPacket.hpp"

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
	len = sizeof(reply_t);
	data = new byte[len];
	packet = (reply_t *) data;
}

CurrentData::CurrentData(){
	len = sizeof(reply_t);
	data = new byte[len];
	packet = (reply_t *) data;
}

/*
template <>
ArduinoCommand<PCdatapacket::command_t>::ArduinoCommand(){
	len = sizeof(PCdatapacket::command_t);
	data = new byte[sizeof(PCdatapacket::command_t)];
	packet = (PCdatapacket::command_t *) data;
}


template <>
ArduinoCommand<PCdatapacket::setint_t>::ArduinoCommand(){
	len = sizeof(PCdatapacket::setint_t);
	data = new byte[sizeof(PCdatapacket::setint_t)];
	packet = (PCdatapacket::setint_t *) data;
}
*/
/*
EncoderData::~EncoderData(){
	packet = NULL;
}
*/

