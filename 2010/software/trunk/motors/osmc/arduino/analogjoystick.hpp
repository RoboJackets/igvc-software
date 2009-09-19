#ifndef ANALOGJOYSTICK_HPP_
#define ANALOGJOYSTICK_HPP_

#include "WProgram.h"
#include "pinDefs.hpp"

#include "motorPWM.hpp"

void setupJoystick();
void getJoystickReading(int* X, int* Y);
void joystickSetMotors();

#endif
