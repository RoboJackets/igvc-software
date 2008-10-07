#ifndef DATA_PACKET_H_
#define DATA_PACKET_H_

#include <iostream>
#include <string>

typedef unsigned char byte;

class PCdatapacket{

	friend class EncoderData;

	public:
		int packnum;
		size_t len;
		byte * data;
		virtual ~PCdatapacket();	
};

class EncoderData : public PCdatapacket{
	public:
		typedef struct __attribute__((__packed__)) { unsigned int timestamp; unsigned int packetnum; short dl; short dr; unsigned short dt; } reply_t;

		reply_t * packet;
		void setDataPointer(void * ptr);

		//~EncoderData();
};

#endif
