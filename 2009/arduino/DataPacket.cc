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

std::ostream& operator<<(std::ostream& output, const PCdatapacket& pk){
	output << "Raw Data\n";
	return(output);
}

EncoderData::EncoderData(){
	len = sizeof(reply_t);
	data = new byte[len];
	packet = (reply_t *) data;
}

std::ostream& operator<<(std::ostream& output, const EncoderData& epk){
	//output << "encoder data\n";
	output << "t: " << epk.packet->timestamp / 1000;
	output << " n: " << epk.packet->packetnum;
	output << " dl: " << epk.packet->dl;
	output << " dr: " << epk.packet->dr;
	output << " dt: " << epk.packet->dt; 
	output << std::endl;
	return(output);
}

CurrentData::CurrentData(){
	len = sizeof(reply_t);
	data = new byte[len];
	packet = (reply_t *) data;
}

std::ostream& operator<<(std::ostream& output, const CurrentData& cpk){
	output << "Current Data\n";
	return(output);
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

