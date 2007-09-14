#ifndef _VISION_NAVIGATION_H_
#define _VISION_NAVIGATION_H_

#include "Buffer2D.h"
#include "Pixel.h"
#include "MotorOutput.h"

// OUTPUT: Visual representation of the navigation parameters.
extern Buffer2D<Pixel> visNavigationParams;

// OUTPUT: Trajectory picked by the vision-navigation algorithm
extern MotorOutput autonomousModeMotorOutput;

// Draws a graphical representation of the current navigation parameters.
void visPlotNavigationParams(void);

#endif // _VISION_NAVIGATION_H_
