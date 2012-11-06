#ifndef CURRENT_SENSORS_HPP_
#define CURRENT_SENSORS_HPP_

#include "WProgram.h"
#include "DataPacketStructs.hpp"
#include "pinDefs.hpp"

current_reply_t getBothCurrentADCVal();
int getLeftCurrentADCVal();
int getRightCurrentADCVal();

#endif
