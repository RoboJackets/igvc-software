#ifndef MOTORPWM_HPP_
#define MOTORPWM_HPP_
#include "WProgram.h"
#include "pinDefs.hpp"
#include "ArduinoCmds.hpp"
#include <math.h>

const int DEAD_ZONE = 100;

void setLeftMotorDutyCycle(byte dir, byte ldc);
void setRightMotorDutyCycle(byte dir, byte rdc);

void setPWMFreq();

#endif
