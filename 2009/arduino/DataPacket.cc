#include "DataPacket.h"

PCdatapacket::~PCdatapacket(){
	delete[] data;
}
