#ifndef MOTORPWM_HPP_
#define MOTORPWM_HPP_
#include "WProgram.h"
#include "pinDefs.hpp"

void setLeftMotorDutyCycle(char ldc);
void setRightMotorDutyCycle(char rdc);

void setPWMFreq();

#endif
