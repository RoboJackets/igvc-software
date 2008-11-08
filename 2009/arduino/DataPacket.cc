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
	output.setf(std::ios::hex);
	output << "dr: " << data.dr << std::endl;
	output << "dl: " << data.dl << std::endl;
	output << "dt: " << data.dt << std::endl;
	return(output);
}

std::ostream& operator<<(std::ostream& output, DataPacket::encoder_reply_t* data){
	output.setf(std::ios::hex);
	output << "dr: " << data->dr << std::endl;
	output << "dl: " << data->dl << std::endl;
	output << "dt: " << data->dt << std::endl;
	return(output);
}
