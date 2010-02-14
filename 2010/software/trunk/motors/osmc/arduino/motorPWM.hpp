#ifndef MOTORPWM_HPP_
#define MOTORPWM_HPP_
#include "WProgram.h"
#include "pinDefs.hpp"
#include "ArduinoCmds.hpp"
#include <math.h>

const byte maxRightDuty = 150;
const byte maxLeftDuty = 150;

//const byte maxRightDuty = 238;
//const byte maxLeftDuty = 255;

const float rightScale = float(maxRightDuty) / float(255);
const float leftScale = float(maxLeftDuty) / float(255);

void setLeftMotorDutyCycle(byte dir, byte ldc);
void setRightMotorDutyCycle(byte dir, byte rdc);

void setPWMFreq();

#endif
