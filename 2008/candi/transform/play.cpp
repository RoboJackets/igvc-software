#include "play.h"

#include <GL/glut.h>
#include <stdlib.h>

#include "ptypes.h"
#include "pglobals.h"
#include "hw/hw.h"
#include "vision/PCamera.h"
#include "vision/vision.h"				// for visFrame
#include "vision/vision_navigation.h"	// for autonomousModeMotorOutput
#include "main.h"						// for mainWindow

// ---------------------------------------------------------------------------------
// Player code

// Connects to the robot
extern "C" void ConnectToRobot() {
	InitHW();
	GetNextFrame();
}

extern "C" Image* GetNextFrame() {
	UpdateSensors();
	
	Image* nextFrame = GetCameraFrame();
	usleep(10);			// don't hog the processor
	return nextFrame;
}

extern "C" void ProcessTransformedFrame(void) {
	// Do vision processing on the transformed frame
	Camera::current = &PCamera::INSTANCE;
	PCamera::INSTANCE.update();
	visFrame();
	
	// Drive the motors using the heading found by the navigation algorithm
	SetMotorOutput(autonomousModeMotorOutput);
	
	// Refresh the main view
	mainWindow->updateVideoView();
}
