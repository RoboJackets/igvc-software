#ifndef DATA_PACKET_H_
#define DATA_PACKET_H_

#include <iostream>
#include <string>

typedef unsigned char byte;

class PCdatapacket{

	//friend class EncoderData;

	public:
		int packnum;
		size_t len;
		byte * data;

		typedef struct __attribute__((__packed__)) { unsigned int timestamp; unsigned int packetnum; } header_t;
		typedef struct __attribute__((__packed__)) { unsigned int timestamp; unsigned int packetnum; char command; } command_t;
		typedef struct __attribute__((__packed__)) { unsigned int timestamp; unsigned int packetnum; char command; unsigned int var; } setint_t;
		virtual ~PCdatapacket();
	//protected:
		PCdatapacket();

		friend std::ostream& operator<<(std::ostream& output, const PCdatapacket& pk);
};

class EncoderData : public PCdatapacket{
	public:
		typedef struct __attribute__((__packed__)) { unsigned int timestamp; unsigned int packetnum; short dl; short dr; unsigned short dt; } reply_t;

		reply_t * packet;
		EncoderData();
		//~EncoderData();

		friend std::ostream& operator<<(std::ostream& output, const EncoderData& epk);
};

class CurrentData : public PCdatapacket{
	public:
		typedef struct __attribute__((__packed__)) { unsigned int timestamp; unsigned int packetnum; short current_l; short current_r; } reply_t;

		reply_t * packet;
		CurrentData();
		//~EncoderData();

		friend std::ostream& operator<<(std::ostream& output, const CurrentData& cpk);
};

#endif
