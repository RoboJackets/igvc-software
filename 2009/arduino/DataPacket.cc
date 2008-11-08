#include "DataPacket.hpp"

DataPacket::DataPacket(){
	data = NULL;
	header.size = 0;
}

DataPacket::~DataPacket(){
	if(data != NULL){
		delete[] data;
	}
}

std::ostream& operator<<(std::ostream& output, DataPacket pk){
	output << "base packet" << std::endl;
	return(output);
}

std::ostream& operator<<(std::ostream& output, DataPacket::encoder_reply_t data){
	//output.setf(std::ios::hex);
	output << std::hex << "dr: " << data.dr << std::endl;
	output << std::hex << "dl: " << data.dl << std::endl;
	output << std::hex << "dt: " << data.dt << std::endl;
	return(output);
}

std::ostream& operator<<(std::ostream& output, DataPacket::encoder_reply_t* data){
	//output.setf(std::ios::hex);
	output << std::hex << "dr: " << data->dr << std::endl;
	output << std::hex << "dl: " << data->dl << std::endl;
	output << std::hex << "dt: " << data->dt << std::endl;
	return(output);
}
