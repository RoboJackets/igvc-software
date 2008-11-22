#ifndef DATAPACKET_HPP_
#define DATAPACKET_HPP_

#include <cstdlib>
#include <iostream>
//length in bytes of header
#define PACKET_HEADER_SIZE 10

typedef unsigned char byte;

class DataPacket{
	public:

	//Header types
	typedef struct __attribute__((__packed__)) { int timestamp; int packetnum; byte cmd; byte size; } header_t;

	//Data types
	typedef struct __attribute__((__packed__)) { int errnum; byte * msg; } error_pk_t;
	typedef struct __attribute__((__packed__)) { short dl; short dr; unsigned int dt;} encoder_reply_t;


	header_t header;
	byte * data;
	//size_t datalen;

	DataPacket();
	~DataPacket();
};

std::ostream& operator<<(std::ostream& output, DataPacket pk);
std::ostream& operator<<(std::ostream& output, DataPacket::encoder_reply_t data);

#endif //DATAPACKET_HPP_
