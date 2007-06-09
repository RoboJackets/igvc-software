#include "play.h"

#include <GL/glut.h>
#include <libplayerc++/playerc++.h>
#include <stdlib.h>

#include "ptypes.h"
#include "pglobals.h"
#include "vision/PCamera.h"
#include "vision/vision.h"				// for visFrame
#include "vision/vision_navigation.h"	// for autonomousModeMotorOutput
#include "main.h"

// ---------------------------------------------------------------------------------
// Player code

static Image curFrame;

static PlayerCc::PlayerClient *robot;
static PlayerCc::CameraProxy *camera;		// NULL if unavailable
static PlayerCc::Position2dProxy *motors;	// NULL if unavailable

static void resizeImage(uint width, uint height);

// Connects to the robot
extern "C" void ConnectToRobot(char *hostname, int port) {
	// Connect to the robot and its devices
	robot = new PlayerCc::PlayerClient(hostname, port);
	try {
		camera = new PlayerCc::CameraProxy(robot,0); 		// use camera:0
	} catch (PlayerCc::PlayerError e) {
		printf("ERROR: Camera not found.\n");
		camera = NULL;
	}
	try {
		motors = new PlayerCc::Position2dProxy(robot,0);	// use position2d:0
	} catch (PlayerCc::PlayerError e) {
		printf("ERROR: Motors not found.\n");
		motors = NULL;
	}
	
	// Set data to PULL mode
	// (i.e., always get the *latest* camera images,
	//  even if some frames are dropped)
	robot->SetDataMode(PLAYER_DATAMODE_PULL);
	robot->SetReplaceRule((bool) 1, -1, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_CODE);
	
	// Read the first frame from the camera
	curFrame.width=0;
	curFrame.height=0;
	curFrame.data=NULL;
	GetNextFrame();
}

extern "C" Image* GetNextFrame() {
	robot->Read();
	
	if (camera != NULL) {
		resizeImage(camera->GetWidth(), camera->GetHeight());
		camera->GetImage(curFrame.data);
	}
	
	usleep(10);
	return &curFrame;
}

// Resizes the image buffer that received images are stored into
void resizeImage(uint width, uint height) {
	if ((width == curFrame.width) && (height==curFrame.height)) return;
	curFrame.width=width;
	curFrame.height=height;
	
	if (curFrame.data) free(curFrame.data);
	curFrame.data = (unsigned char *) malloc(width*height*3);
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

void SetMotorOutput(MotorOutput motorOutput) {
	if (motors != NULL) {
		motors->SetSpeed(
			(double) motorOutput.leftSpeed*2,		// output: -255 to 255
			(double) motorOutput.rightSpeed*2);		// output: -255 to 255
	}
}
