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
	len = sizeof(reply_t);
	data = new byte[sizeof(reply_t)];
	packet = (reply_t *) data;
}

CurrentData::CurrentData(){
	len = sizeof(reply_t);
	data = new byte[sizeof(reply_t)];
	packet = (reply_t *) data;
}

template <typename T> 
ArduinoCommand<T>::ArduinoCommand(){
	len = sizeof(T);
	data = new byte[sizeof(T)];
	packet = (T *) data;
}

template <>
ArduinoCommand<PCdatapacket::command_t>::ArduinoCommand(){
	len = sizeof(PCdatapacket::command_t);
	data = new byte[sizeof(PCdatapacket::command_t)];
	packet = (PCdatapacket::command_t *) data;
}

/*
EncoderData::~EncoderData(){
	packet = NULL;
}
*/
