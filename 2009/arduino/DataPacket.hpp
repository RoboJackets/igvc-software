#ifndef DATAPACKET_HPP_
#define DATAPACKET_HPP_

#include <cstdlib>
#include <iostream>
//length in bytes of header
#define PACKET_HEADER_SIZE 10

#define PACKET_ERROR_CMD 0xFF

//1 byte commands, laptop -> arduino
#define ARDUINO_GETSTATUS_CMD 'r'
#define ARDUINO_SETVAR_CMD 'w'
#define ARDUINO_ID_CMD 'i'
#define ARDUINO_RSND_PK_CMD 'p'

//1 byte commands/response arduino -> laptop
#define ARDUINO_ERROR 0xFF
#define ARDUINO_GETSTATUS_RESP 'r'
#define ARDUINO_SETVAR_RESP 'w'
#define ARDUINO_ID_RESP 'i'
#define ARDUINO_RSND_PK_RESP 'p'

typedef unsigned char byte;

enum ARDUINO_ERROR_STATUS { DROPPED_PACKET, REQUESTED_PACKET_OUT_OF_RANGE };

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
