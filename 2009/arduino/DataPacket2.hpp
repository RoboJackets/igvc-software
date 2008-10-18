#ifndef DATA_PACKET2_H_
#define DATA_PACKET2_H_

#include "DataPacket.hpp"

#include <iostream>
#include <string>

typedef unsigned char byte;

template <typename T> 
class ArduinoCommand : public PCdatapacket{
	public:
		//typedef struct __attribute__((__packed__)) { unsigned int timestamp; unsigned int packetnum; char command; } command_t;
		T * packet;

		ArduinoCommand();
};


template <typename T> 
ArduinoCommand<T>::ArduinoCommand(){
	len = sizeof(T);
	data = new byte[len];
	packet = (T *) data;
}


#endif
