#ifndef DATA_PACKET_H_
#define DATA_PACKET_H_

#include <iostream>
#include <string>

typedef unsigned char byte;

class PCdatapacket{
	public:
		int packnum;
		size_t len;
		byte * data;
		~PCdatapacket();
};

#endif
