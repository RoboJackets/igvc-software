#ifndef HW_H
#define HW_H

#include "transform/ptypes.h"	// for Image	// TODO: move to this package
#include "vision/MotorOutput.h"		// TODO: move to this package

// Whether or not to keep looking for devices after the initial connect.
#define SUPPORT_PLUG_AND_PLAY 0

void InitHW();
void UpdateSensors();

Image* GetCameraFrame();

void SetMotorOutput(MotorOutput motorOutput);

/**
 * Returns the range (in meters) of the obstacle at the specified
 * bearing (in radians), or -1 if an error occurred.
 */
double GetLaserRangeAtBearing(double bearing);

#endif
