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
#include "opencv/highgui.h"
#include "Point2D.h"
#include "vision.h"
#include "image_buffers.h"
#include "mapgen.h"
#include "joystickDrive.hpp"

const static double motor_vel_mag = 1.1;

#ifdef OSMC_2WD
	#include "OSMC_driver.hpp"	
#elif defined(OSMC_4WD)
	#include "OSMC_4wd_driver.hpp"	
#else
	#error "Must define OSMC_2WD or OSMC_4WD"
#endif

//#include "NAV200.hpp"

/*===== Camera Settings ===========================================*/
/* Determines camera compatability.
 * This must be set to 0 to load videos via command line. 
 * Note -- this is dead, use config file
 */
//#define USE_FIREWIRE_CAMERA 0
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
	GuppyCam camera_firewire;
	CVcam camera_usb;

	/* the vision processing object */
	Vision vp;
	int doVision;
	int doTransform;

	// initializes various things before main loop (mainly CV images)
	int init();

	// cleans up image buffers
	void releaseAllImages();

	// runs the robot (loops processFunc)
	void Go();

	// MAIN LOOP
	void processFunc();

	// initializes opengl and glut
	void initGlut();

	// refreshes glut window with raw image data
	void updateGlutDisplay();

	// gets image mask to mark void transform area
	void getGlutMask(int call);

	// starts the robot
	void startRobotThread(void* obj);

	/* the robot thread
	 * this is static so "atexit()" works */
	static pthread_t robotThread;

	// connects to the camera (USB or 1394)
	void connectToCamera();
	// sorry to hack camera sorce this way...
	int USE_FIREWIRE_CAMERA;
	// to load a video
	std::string videofilename;

	// gets raw image
	int GrabImage();

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
	Point2D<int> heading_mapping;

	// avg heading control
	// k_motors = the % of new value to use
	float k_motors;

	// xml conf
	void LoadXMLSettings();

	// map generator and slam processing
	MapGen mapper;
	int doMapping;

	// motor control
	//Motors_Old motors;
	int motorsMaxSpeed;//0-255
	int useMotors;

	#ifdef OSMC_2WD
		OSMC_driver* osmcd;
	#elif defined(OSMC_4WD)
		OSMC_4wd_driver* osmcd;
	#else
		#error "Must define OSMC_2WD or OSMC_4WD"
	#endif
	boost::mutex velmutex;
	volatile double yvel;
	volatile double xvel;
	volatile bool run_vel_thread;
	boost::thread* vel_update_thread;
	void update_vel_func();
	

	//NAV200* lidar;
	boost::mutex lidarmutex;
	volatile bool run_lidar_thread;
	boost::thread* lidar_update_thread;
	void update_lidar_func();

	// joystick driver
	joystickDrive jD;

	//radians <start,stop>; 0 to the right, pos counterclockwise
	//std::deque< boost::tuple<float,float> > lidar_linear_regions;
	//boost::tuple<float,float> longest_linear_region;
	//boost::tuple<float,float> coord[NAV200::Num_Points];
};

#endif /*ROBOT_H_*/

