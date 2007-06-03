#include <GL/glut.h>
#include <libplayerc++/playerc++.h>
#include <stdlib.h>
#include "ptypes.h"
#include "pglobals.h"
#include "vision/PCamera.h"
#include "vision/vision.h"
#include "main.h"

Image *im1 = NULL;

// ---------------------------------------------------------------------------------
// Player code

PlayerCc::PlayerClient *robot;
PlayerCc::CameraProxy *camera;
void resizeImage(int width, int height);

// Connects to the robot
extern "C" void ConnectToRobot(char *hostname, int port) {
	robot = new PlayerCc::PlayerClient(hostname, port);
	camera = new PlayerCc::CameraProxy(robot,0); // use camera:0
	
	// Set data to PULL mode
	// (i.e., always get the *latest* camera images,
	//  even if some frames are dropped)
	robot->SetDataMode(PLAYER_DATAMODE_PULL);
	robot->SetReplaceRule((bool) 1, -1, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_CODE);

	im1 = (Image*) malloc(sizeof(Image));
	im1->width=0;
	im1->height=0;
    im1->data=NULL;
	robot->Read();
	resizeImage(camera->GetWidth(), camera->GetHeight());
	camera->GetImage(im1->data);
	
	Camera::current = &PCamera::INSTANCE;
}

extern "C" Image* GetNextFrame() {
	robot->Read();
	resizeImage(camera->GetWidth(), camera->GetHeight());
	camera->GetImage(im1->data);
	usleep(10);
	return im1;
	
}

// Resizes the image buffer that received images are stored into
void resizeImage(int width, int height) {
	if ((width == im1->width) && (height==im1->height)) return;
	im1->width=width;
	im1->height=height;
	
	if (im1->data) free(im1->data);
	im1->data = (unsigned char *) malloc(width*height*3);
}

extern "C" void ProcessTransformedFrame(void) {
	PCamera::INSTANCE.update();
	visFrame();
	mainWindow->updateVideoView();
}
