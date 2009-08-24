#ifndef CURRENT_SENSORS_HPP_
#define CURRENT_SENSORS_HPP_

#include "WProgram.h"
#include "DataPacketStructs.hpp"
const int leftCurrentADCPin	= 3;
const int rightCurrentADCPin	= 4;

current_reply_t getBothCurrentADCVal();
int getLeftCurrentADCVal();
int getRightCurrentADCVal();

#endif
