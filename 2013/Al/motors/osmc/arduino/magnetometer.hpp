#ifndef CURRENT_MAGNET_HPP_
#define CURRENT_MAGNET_HPP_

#include "WProgram.h"
#include "DataPacketStructs.hpp"
#include "pinDefs.hpp"

magnetometer_pk_t getHeading();
int ShiftIn(int);
void ShiftOut(int, int);
void HM55B_Reset();
void HM55B_StartMeasurementCommand();
int HM55B_ReadCommand();
void setup();

#endif
