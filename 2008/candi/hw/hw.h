#ifndef HW_H
#define HW_H

#include "transform/ptypes.h"	// for Image	// TODO: move to this package
#include "vision/MotorOutput.h"		// TODO: move to this package

// Whether or not to keep looking for devices after the initial connect.
#define SUPPORT_PLUG_AND_PLAY 0

#define LOOK_FOR_REMOTE_RC 0

void InitHW();
void UpdateSensors();

/**
 * Blocks until a new image from the camera is available and then returns it.
 * <p>
 * The returned image is stored in a static buffer,
 * so users do not need to (and should not) deallocate it.
 */
Image* GetCameraFrame();

void SetMotorOutput(MotorOutput motorOutput);

/**
 * Returns the range (in meters) of the obstacle at the specified
 * bearing (in radians), or -1 if an error occurred.
 */
double GetLaserRangeAtBearing(double bearing);

#endif
