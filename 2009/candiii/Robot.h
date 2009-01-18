#ifndef ROBOT_H_
#define ROBOT_H_

/*
 * This file represents the robot and all its functions
 *   by: Chris McClanahan
 *
 */

#include "CVcam.h"
#include "GuppyCam.h"
#include <stdlib.h>
#include "highgui.h"
#include "Point2D.h"
#include "vision.h"

#include "mapgen.h"

/*===== Camera Settings ===========================================*/
/* Determines camera compatability.
 * This must be set to 0 to load videos via command line. */
#define USE_FIREWIRE_CAMERA 0

/*=================================================================*/

class Robot
{
public:
	// default constructor, use a camera
	Robot();

	// constructor for loading a camera, or a video file
	Robot(const char* filename);

	// destructor
	virtual ~Robot();

	/* Cleans up any resources that were allocated
	 * Init will have this called on atexit (stdlib.h) so the
	 * user does not need to call this */
	static void destroy();

	/* the vision source */
#if USE_FIREWIRE_CAMERA

	GuppyCam camera;
#else

	CVcam camera;
#endif

	/* the vision processing object */
	Vision vp;

	// initializes various things before main loop (mainly CV images)
	int init();

	// runs the robot (loops processFunc)
	void Go();

	// MAIN LOOP
	void processFunc();

	// initializes opengl and glut
	void initGlut();

	// refreshes glut window with raw image data
	void updateGlutDisplay();

	// starts the robot
	void startRobotThread(void* obj);

	/* the robot thread
	 * this is static so "atexit()" works */
	static pthread_t robotThread;

	// connects to the camera (USB or 1394)
	void connectToCamera();

	// view to display
	int trackbarVal;

	// structure for saving video
	CvVideoWriter* cvVideoWriter;
	void createVideoWriter();

	// velocity headings for robot
	// heading.x = rotational speed
	// heading.y = forward speed
	Point2D<int> heading_vision;
	Point2D<int> heading_sensors;
	Point2D<int> heading_main;

	// avg heading control
	// _k = the % of new value to use
	double _k;

	// xml conf
	void LoadXMLSettings();

	// map generator and slam processing
	MapGen mg;

};

#endif /*ROBOT_H_*/

