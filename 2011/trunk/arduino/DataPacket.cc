#include "DataPacket.hpp"

DataPacket::DataPacket()
{
	data = NULL;
	header.size = 0;
}

DataPacket::~DataPacket()
{
	if (data != NULL)
	{
		delete[] data;
		data = NULL;
		header.size = 0;
	}
}

DataPacket::DataPacket(const DataPacket& pk)
{
	this->header = pk.header;
	this->data = new byte[pk.header.size];
	memcpy(this->data, pk.data, pk.header.size);

	//std::cout << "copy constructor called" << std::endl;
}

void DataPacket::clear()
{
	header.timestamp_sec = -1;
	header.timestamp_usec = -1;
	header.packetnum = 0;
	header.cmd = 0;
	header.size = 0;

	if (data != NULL)
	{
		delete[] data;
		data = NULL;
		header.size = 0;
	}
}

std::ostream& operator<<(std::ostream& output, DataPacket& pk)
{
//	output << std::hex;
	output << "timestamp sec: " << pk.header.timestamp_sec << std::endl;
	output << "timestamp usec: " << pk.header.timestamp_usec << std::endl;
//	output << std::dec;
	output << "packetnum: " << pk.header.packetnum << std::endl;
	output << "cmd (char): " << (char) pk.header.cmd << "\tcmd (ascii): " << (unsigned int) pk.header.cmd << std::endl;
	output << "size: " << (int) pk.header.size << std::endl;

	return(output);
}

std::ostream& operator<<(std::ostream& output, header_t header)
{
//	output << std::hex;
	output << "timestamp sec: " << header.timestamp_sec << std::endl;
	output << "timestamp usec: " << header.timestamp_usec << std::endl;
//	output << std::dec;
	output << "packetnum: " << header.packetnum << std::endl;
	output << "cmd (char): " << (char) header.cmd << "\tcmd (ascii): " << (unsigned int) header.cmd << std::endl;
	output << "size: " << (int) header.size << std::endl;

	byte* header_p = (byte*) &header;
	output << std::hex << "bin: ";
	for(size_t i = 0; i < sizeof(header_t); i++)
	{
		output << int(header_p[i]) << ", ";
	}
	output << std::dec;
	return(output);
}

std::ostream& operator<<(std::ostream& output, encoder_reply_t data)
{
	//output.setf(std::ios::hex);

	output << std::hex;

	output << "dr: " << data.dr << std::endl;
	output << "dl: " << data.dl << std::endl;
	output << "dt: " << data.dt << std::endl;

	output << std::dec;

	return(output);
}

std::ostream& operator<<(std::ostream& os, const new_encoder_pk_t& rv)
{
	os << "l: " << rv.pl << std::endl;
	os << "r: " << rv.pr << std::endl;
	os << "dl: " << rv.dl << std::endl;
	os << std::hex;
	os << "dr: " << rv.dr << std::endl;
	os << std::dec;
	return os;
}

std::ostream& operator<<(std::ostream& os, const new_encoder_single_pk_t& rv)
{
	os << "pos: " << rv.pcoder << std::endl;
	os << "dpos: " << rv.dcoder << std::endl;
	os << std::dec;
	return os;
}

/*
std::ostream& operator<<(std::ostream& output, DataPacket::encoder_reply_t* data){
	//output.setf(std::ios::hex);
	output << std::hex << "timestamp: " << header->timestamp << std::endl;
	output << std::hex << "packetnum: " << header->packetnum << std::endl;
	output << std::hex << "cmd: " << header.cmd << std::endl;
	output << std::hex << "size: " << header.size << std::endl;


	output << std::hex << "dr: " << data->dr << std::endl;
	output << std::hex << "dl: " << data->dl << std::endl;
	output << std::hex << "dt: " << data->dt << std::endl;

	output << std::dec;
	return(output);
}
*/
