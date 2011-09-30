#include <sstream>

#include "Robot.h"
#include "vision.h"
#include "main.h"
#include "image_buffers.h"
#include "PjMat.h"
#include <stdlib.h>
#include <GL/glut.h>
#include "XmlConfiguration.h"
#include "logging/timer.h"


#define PRINTFRAMERATE 0


//#define STOPANDTHINK

/*********** GLUT callbacks and functions ***********************/
pthread_t Robot::robotThread; // for pthread_create
Robot* glRobot; // for glut
static GLuint cameraImageTextureID; // for glut
int glutwindow; // for glut
int saveRawVideo; // flag for saving video - global because of glut use
void* robot_thread_caller(void* arg)
{
	saveRawVideo=0; // don't save video yet
	glRobot = (Robot*)arg; // assign pointer to static robot object for glut to use
	static_cast<Robot*>(arg)->Go(); // start the robot
	glRobot->destroy(); // kill the robot;
	return NULL;
}
void robot_process_function_caller(void)
{
	glRobot->processFunc(); // for glutMainLoop
}
void idleFunc(void)   // for refresheing glut window
{
	glutPostRedisplay();
}
void keyboardFunc(unsigned char key, int x, int y)   // handles keyboard button presses
{
	switch (key)
	{
	case 'd':
		printf("die! \n");
		glRobot->destroy(); // kill the robot;
		break;
	case 'D':
		printf("die! \n");
		glRobot->destroy(); // kill the robot;
		break;
	case 's':
		//saveRawVideo = 1-saveRawVideo; // save video until s again
		saveRawVideo = 1; // save untill  program exit
		printf("video file \n");
		break;
	case 'p':
		printf("PAUSED!");
		cvWaitKey(0); // pause
		printf("GO!");
		break;
	default:
		printf("x,y %d,%d \n",x,y);
		break;
	}
}
/****************************************************************/

/********** CV window callback stuff *****************/
// callback for trackbar
/* for selecting images to display in the opencv window */
void trackbarHandler(int pos)
{
	printf("\tview %d \n", pos);
}
/*****************************************************/


// constructor
Robot::Robot(const char* filename)
{
	/* only the CVcam can load video... */
	if (filename != NULL)
		videofilename = filename;
	else
		videofilename = "";

}

Robot::~Robot()
{
	osmcd->setLight(MC_LIGHT_STEADY);

	run_vel_thread = false;
	run_lidar_thread = false;
	vel_update_thread->join();
	lidar_update_thread->join();
	delete vel_update_thread;
	delete lidar_update_thread;
	delete osmcd;
	//delete lidar;

	cvReleaseVideoWriter(&cvVideoWriter);
	ImageBufferManager::getInstance().releaseAllImages();
	destroy();
}

void Robot::destroy()
{
	/* clean up */
	glutDestroyWindow(glutwindow);
	cvDestroyAllWindows();
	exit(0);
}

int Robot::init()
{
	//set vel to 0
	{
		boost::mutex::scoped_lock lock(velmutex);
		yvel = 0;
		xvel = 0;
	}

	/* load xml settings - important to do first! */
	LoadXMLSettings();

	/* select and connect to the camera source ,
	 *   and possibly load a video */
	if (USE_FIREWIRE_CAMERA)
	{
		/* connect to the camera */
		connectToCamera();
	}
	else
	{
		if (videofilename=="")
		{
			/* connect to the camera */
			connectToCamera();
		}
		else
		{
			/* load a video */
			camera_usb.connect(0, videofilename.c_str());
		}
	}

	/* try to grab a frame to get image size */
	if (USE_FIREWIRE_CAMERA)
	{
		if (!camera_firewire.GrabCvImage())
		{
			printf("Error getting camera frame \n");
			return 0; // fail
		}
	}
	else
	{
		if (!camera_usb.GrabCvImage())
		{
			printf("Error getting camera frame \n");
			return 0; // fail
		}
	}

	/* setup image selection bar */
	int numberOfViews = 15; // important!!!
	cvCreateTrackbar("bar","display",&trackbarVal,numberOfViews,trackbarHandler);

	/* configure opencv display window */
	cvResizeWindow( "display", 
			ImageBufferManager::getInstance().getvisCvRaw()->width-30, 
			ImageBufferManager::getInstance().getvisCvRaw()->height+10 );
	cvMoveWindow( "display", 5, 10 ); // position on screen

	/* Init default view (debug=1) */
	trackbarHandler( trackbarVal );

	// EDFU remove this!!
	/* init all CV images here */
	{
		ImageBufferManager::getInstance().init();
	}

	/* set cleanup on exit */
	atexit(Robot::destroy);

	/* init video writer */
	//createVideoWriter();   // mising codecs?

	/* setup vision module */
	vp.init();

	/* setup slam processing module */
	mapper.init();

	/* connect to motors */
	if (useMotors)
	{
		//deprecated - old motors
		#if 0
		motors.set_max_speed(motorsMaxSpeed);
		if (0!=motors.SetupSerial())
		{
			printf("Motors Connect Fail !\n");
			return 0; // fail
		}
		#endif
	}

	/* success */
	return 1;
}

void Robot::LoadXMLSettings()
{
	/* load xml file */
	XmlConfiguration cfg("Config.xml");

	/* load settings */
	{
		/* k value = the % of new value to use */
		k_motors = cfg.getFloat("k_motors");
		/* see ConvertAllImageViews() in vision.cc */
		trackbarVal = cfg.getInt("defaultView");
		/* 0-255 */
		motorsMaxSpeed = cfg.getInt("motorsMaxSpeed");
		/* SLAM */
		doMapping = cfg.getInt("doMapping");
		/* drive motors */
		useMotors = cfg.getInt("useMotors");
		/* vision processing */
		doVision = cfg.getInt("doVision");
		/* use gl transformation */
		doTransform = cfg.getInt("doTransform");
		/* camera source */
		USE_FIREWIRE_CAMERA = cfg.getInt("useFirewireCamera");

	}

	/* test */
	{
		if (doVision==-1 || trackbarVal==-1)
		{
			printf("ERROR: Robot settings NOT loaded! Using DEFAULTS \n");
			{
				// load defaults
				k_motors = .30;
				trackbarVal = 1;
				motorsMaxSpeed = 30;
				doMapping = 1;
				useMotors = 1;
				doVision = 1;
				doTransform = 1;
				USE_FIREWIRE_CAMERA = 1;
			}
		}
		else
		{
			printf("Robot settings loaded \n");
		}
		printf("\tvalues: k_motors %f view %d motors %d \n",k_motors,trackbarVal,motorsMaxSpeed);
	}

}

void Robot::Go()
{
	/*
	 * This function initializes and starts the robot
	 */

	/* Quit if we can't initialize properly */
	if (!init())
		return;

	/* Quit if there is no camera */
	if (USE_FIREWIRE_CAMERA)
	{
		if (!camera_firewire.isValid())
		{
			return; // fail
		}
	}
	else
	{
		if (!camera_usb.isValid())
		{
			return; // fail
		}
	}

	#ifdef OSMC_2WD
		osmcd = new OSMC_driver;
	#elif defined(OSMC_4WD)
		osmcd = new OSMC_4wd_driver;
	#else
		#error "Must define OSMC_2WD or OSMC_4WD"
	#endif
	osmcd->setLight(MC_LIGHT_PULSING);


	run_vel_thread = true;
	vel_update_thread = new boost::thread(&Robot::update_vel_func, this);

	//lidar = new NAV200;
	run_lidar_thread = true;
	lidar_update_thread = new boost::thread(&Robot::update_lidar_func, this);

	/* Setup video card processing */
	initGlut();


	/*
	 * Robot Loop!
	 */
	glutMainLoop(); // runs processFunc()

}

/*
 * The main processing function
 */
void Robot::processFunc()
{
	/*
	 * This function should be just simple function calls.
	 */

	/*
	 * Heading Format:
	 * x = rotational speed ; range = (-128,127)
	 * y = forward speed    ; range = (0,255)
	 */

	// Switches into joystick mode if manual override button is pressed
	/*
	jD.readJoystick();
	if (jD.manualOverride())
	{
		osmcd->setLight(MC_LIGHT_STEADY);
		while (jD.manualOverride())		
		{
			jD.setMotor();
			usleep(1e5);
			jD.readJoystick();
		}	
	
		osmcd->setLight(MC_LIGHT_PULSING);
	}
	*/
	/* glut mask init hack */
	static bool getMask = true;
	if (getMask)
	{
		getMask=false;
		/* get transform edge mask */
		getGlutMask(1);
		getGlutMask(2);
		getGlutMask(3);
		getGlutMask(4);
	}


	/* Get raw image */
	if (!GrabImage())
		return;


	/* Shove raw image into graphics card for some processing on the card */
	updateGlutDisplay();


	/* Scale raw image down to 320x240 */
	cvResize(	ImageBufferManager::getInstance().getvisCvRawTransform(), 
			ImageBufferManager::getInstance().getvisCvRawTransformSmall(), 
			CV_INTER_LINEAR);


	/* Perform vision processing. */
	if (doVision)
	{
		vp.visProcessFrame(heading_vision);
	}


	/* Perform SLAM Processing */
	if (doMapping)
	{
		if ( mapper.genMap() )
		{
			mapper.processMap(heading_mapping);
		}
	}


	/* Update displays */
	vp.ConvertAllImageViews(trackbarVal); // display views based on trackbar position


	/* Drive Robot via motor commands (GO!) */
	if (useMotors)
	{
		if (doMapping)
		{
			/* drive via mapping */
			/* Average motor commands from vision (account for high frame rate)
			 * k = % of new value to use */
			heading_main.x = k_motors*heading_mapping.x + (1-k_motors)*heading_main.x;
			heading_main.y = k_motors*heading_mapping.y + (1-k_motors)*heading_main.y;
		}
		else
		{
			/* drive via vision */
			/* Average motor commands from vision (account for high frame rate)
			 * k = % of new value to use */
			heading_main.x = k_motors*heading_vision.x + (1-k_motors)*heading_main.x;
			heading_main.y = k_motors*heading_vision.y + (1-k_motors)*heading_main.y;
		}
	}

#ifdef STOPANDTHINK
	/* periodically stop and think */
	static int hack = 0;
	int hackstop = 76; //80;

	if ( hack > hackstop )
	{
		heading_main.x = 0;
		heading_main.y = 0;
		//hack = 0;
		printf("\n\n\n\tSTOP HACK\t\n\n\n");
		//motors.set_motors(-20, -20);
		//motors.set_motors(hack/8, hack/8);
		{
			boost::mutex::scoped_lock lock(velmutex);
			yvel = 0;
			xvel = 0;
			//osmcd->set_motors(hack, hack);//will get picked up on next update
		}
		if (hack > hackstop+14 )
		{
			hack = 0;
			printf("\n\n\n\tRESUME!!!!!!\t\n\n\n");
		}
		++hack;
		if (!ON_RAMP)
		{
			return;// causes robot to go backwards, otherwise it just stops on ramp
		}
	}
	++hack;

//	if(DO_STOPANDTHINK)
//	{
//		heading_main.x = 0;
//		heading_main.y = 0;
//		DO_STOPANDTHINK = 0;
//		printf("\n\n\n\tSTOP HACK\t\n\n\n");
//		motors.set_motors(0, 0);
//		printf("\n\n\n\tRESUME!!!!!!\t\n\n\n");
//    }
#endif



	/*Save raw image last*/
	static int imgnum = 0;
#ifdef STOPANDTHINK
	if (saveRawVideo && (hack%2==0) )
#else
	if (saveRawVideo )
#endif
	{

		std::stringstream fname;
		fname << "./images/im_" << imgnum << ".jpg";

		cvSaveImage(fname.str().c_str(), ImageBufferManager::getInstance().getvisCvRaw());
		++imgnum;
	}


	/* Drive Motors */
	if (useMotors)
	{
		double mag = hypot(heading_main.x, heading_main.y);
		double unit_x = heading_main.x / mag;
		double unit_y = heading_main.y / mag;
		double scale_x = unit_x * motor_vel_mag; //set to 1.5 m/s max
		double scale_y = unit_y * motor_vel_mag; //set to 1.5 m/s max
		//std::cout << "DBG: sx: " << unit_x << " sy: " << unit_y << "mag: " << mag << " angle from y:" << angle << std::endl;
		//printf("Heading: rot: %d  fwd: %d \n",heading_main.x,heading_main.y);

		//set_heading(heading_main.y, heading_main.x);
		{
			boost::mutex::scoped_lock lock(velmutex);
			yvel = scale_y;
			xvel = scale_x;
		}
	}

	/* Print Stats */
	if (PRINTFRAMERATE)
	{
		printf( "framerate: %.2f \n\n", elapsed_time() );
		start_timer(); // called second to time entire process (except first run)
	}

	/* pause (for testing) */
	//cvWaitKey(0);
}

void Robot::startRobotThread(void* obj)
{
	//sleep(.5);
	robot_thread_caller(obj);
}

void Robot::connectToCamera()
{
	if (USE_FIREWIRE_CAMERA)
	{
		if (!camera_firewire.connect())
		{
			printf("Camera connect failure \n");
			printf("Try using sudo... \n");
			exit(-1);
		}
		else
		{
			camera_firewire.loadSettings();
			printf("Camera settings loaded \n");
		}
	}
	else
	{
		if (!camera_usb.connect())
		{
			printf("Camera connect failure \n");
			printf("Try using sudo... \n");
			exit(-1);
		}
		else
		{
			camera_usb.loadSettings();
			printf("Camera settings loaded \n");
		}
	}

}

int Robot::GrabImage()
{
	if (USE_FIREWIRE_CAMERA)
	{
		if (!camera_firewire.GrabCvImage())
		{
			printf("Error getting camera frame \n");
			return 0; // fail
		}
	}
	else
	{
		if (!camera_usb.GrabCvImage())
		{
			printf("Error getting camera frame \n");
			return 0; // fail
		}
	}

	return 1;
}

void Robot::initGlut()
{
	// dummy args
	int argc = 0;
	char** argv = NULL;

	// initialization
	glutInit(&argc, argv);
	glutInitWindowSize(	ImageBufferManager::getInstance().getvisCvRaw()->width, 
				ImageBufferManager::getInstance().getvisCvRaw()->height);
	//glutInitWindowPosition(800, 480); 				// position on screen
	glutInitWindowPosition(0, 480); 				// position on screen
	glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
	glutwindow = glutCreateWindow("Transform");
	glutDisplayFunc(robot_process_function_caller); // the function glutMainLoop() runs
	glutIdleFunc(idleFunc);							// refreshes the glut window
	glutKeyboardFunc(keyboardFunc);
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(1, &cameraImageTextureID);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraImageTextureID);
	glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glClearColor( 0.0, 0.0, 0.0, 1.0 ); // set edge voidness to black

	// previously in updateGlutDisplay()
	{
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		if (doTransform) setPjMat(); // create transform matrix
	}

	/* get transform edge mask */
	getGlutMask(0); // zero here for first call to function
}



void Robot::updateGlutDisplay()
{

	if (doTransform)
	{
		/* perspective transform */

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//glDisable(GL_DEPTH_TEST);
		//glDisable(GL_LIGHTING);
		//glEnable(GL_TEXTURE_RECTANGLE_ARB);
		{

			/* * * transform * * */
			//glLoadIdentity ();
			//glOrtho (-1.0, 1.0, -1.0, 1.0, -1.0, 1.0); // sets up basic scale for input for you to draw on
			//glMatrixMode(GL_PROJECTION);
			//glLoadIdentity ();
			/* * * * * * * * * * */

			/* put data in card */
			glTexImage2D(	GL_TEXTURE_RECTANGLE_ARB, 
					0, 
					GL_RGB, 
					ImageBufferManager::getInstance().getvisCvRaw()->width, 
					ImageBufferManager::getInstance().getvisCvRaw()->height, 
					0, 
					GL_BGR, 
					GL_UNSIGNED_BYTE, 
					ImageBufferManager::getInstance().getvisCvRaw()->imageData);

			/* * * transform * * */
			//setPjMat();
			/* * * * * * * * * * */

			glBegin(GL_QUADS);
			{

				/* * * transform * * */
				glTexCoord2i(0, 				0);
				glVertex3f(-1,	-1,	0);
				glTexCoord2i(ImageBufferManager::getInstance().getvisCvRaw()->width, 	0);
				glVertex3f( 1,	-1,	0);
				glTexCoord2i(	ImageBufferManager::getInstance().getvisCvRaw()->width, 	
						ImageBufferManager::getInstance().getvisCvRaw()->height);
				glVertex3f( 1,   1,	0);
				glTexCoord2i(0, ImageBufferManager::getInstance().getvisCvRaw()->height);
				glVertex3f(-1,	 1,	0);
				/* * * * * * * * * * */

			}
			glEnd();

		}
		//glDisable(GL_TEXTURE_RECTANGLE_ARB);

		//glFinish();
		//glFlush();

		/* get data from card */
		glReadPixels( 0				,	//GLint x,
					  0				,	//GLint y,
					  ImageBufferManager::getInstance().getvisCvRaw()->width	,	//GLsizei width,
					  ImageBufferManager::getInstance().getvisCvRaw()->height,	//GLsizei height,
					  GL_BGR			,	//GLenum format,
					  GL_UNSIGNED_BYTE,	//GLenum type,
					  //visCvRaw->imageData //Image
					  //visCvDebug->imageData //Image
					  ImageBufferManager::getInstance().getvisCvRawTransform()->imageData //Image
					);

		// double buffering
		glutSwapBuffers();

	}
	else
	{
		/* no transformation */

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		{

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			gluOrtho2D(	0.0, 
					(GLdouble)ImageBufferManager::getInstance().getvisCvRaw()->width,	
					0.0, 
					(GLdouble)ImageBufferManager::getInstance().getvisCvRaw()->height);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glMatrixMode(GL_MODELVIEW);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraImageTextureID);
			/* put data in card */
			glTexImage2D(	GL_TEXTURE_RECTANGLE_ARB, 
					0, 
					GL_RGB, 
					ImageBufferManager::getInstance().getvisCvRaw()->width, 
					ImageBufferManager::getInstance().getvisCvRaw()->height, 
					0, 
					GL_BGR, 
					GL_UNSIGNED_BYTE, 
					ImageBufferManager::getInstance().getvisCvRaw()->imageData);
			glBegin(GL_QUADS);
			{

				// default perspective (upside down)
				glTexCoord2i(0,					0);
				glVertex2i(0, 				0);
				glTexCoord2i(ImageBufferManager::getInstance().getvisCvRaw()->width, 	0);
				glVertex2i(ImageBufferManager::getInstance().getvisCvRaw()->width, 0);
				glTexCoord2i(	ImageBufferManager::getInstance().getvisCvRaw()->width, 	
						ImageBufferManager::getInstance().getvisCvRaw()->height);
				glVertex2i(	ImageBufferManager::getInstance().getvisCvRaw()->width, 
						ImageBufferManager::getInstance().getvisCvRaw()->height);
				glTexCoord2i(0, ImageBufferManager::getInstance().getvisCvRaw()->height);
				glVertex2i(0, 	ImageBufferManager::getInstance().getvisCvRaw()->height);
				// corrected perspective (normal)
				//glTexCoord2i(0, 				visCvRaw->height);	glVertex2i(0, 				0);
				//glTexCoord2i(visCvRaw->width, 	visCvRaw->height);	glVertex2i(visCvRaw->width, 0);
				//glTexCoord2i(visCvRaw->width, 	0);					glVertex2i(visCvRaw->width, visCvRaw->height);
				//glTexCoord2i(0, 				0); 				glVertex2i(0, 				visCvRaw->height);

			}
			glEnd();

		}
		glDisable(GL_TEXTURE_RECTANGLE_ARB);

		/* get data from card */
		//glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, visCvRaw->imageData);
		glReadPixels( 0				,	//GLint x,
					  0				,	//GLint y,
					  ImageBufferManager::getInstance().getvisCvRaw()->width	,	//GLsizei width,
					  ImageBufferManager::getInstance().getvisCvRaw()->height,	//GLsizei height,
					  GL_BGR			,	//GLenum format,
					  GL_UNSIGNED_BYTE,	//GLenum type,
					  //visCvRaw->imageData //Image
					  //visCvDebug->imageData //Image
					  ImageBufferManager::getInstance().getvisCvRawTransform()->imageData //Image
					);

		// double buffering
		glutSwapBuffers();
	}

} // end update glut


//void Robot::createVideoWriter() {
//    cvVideoWriter = 0;
//    int isColor = 1;
//    int fps     = 10;  // or 30
//    int frameW  = visCvRaw->width;
//    int frameH  = visCvRaw->height;
//    /*	Other possible codec codes:
//    	CV_FOURCC('P','I','M','1')    = MPEG-1 codec
//    	CV_FOURCC('M','J','P','G')    = motion-jpeg codec (does not work well)
//    	CV_FOURCC('M', 'P', '4', '2') = MPEG-4.2 codec
//    	CV_FOURCC('D', 'I', 'V', '3') = MPEG-4.3 codec
//    	CV_FOURCC('D', 'I', 'V', 'X') = MPEG-4 codec
//    	CV_FOURCC('U', '2', '6', '3') = H263 codec
//    	CV_FOURCC('I', '2', '6', '3') = H263I codec
//    	CV_FOURCC('F', 'L', 'V', '1') = FLV1 codec
//    */
//    cvVideoWriter=cvCreateVideoWriter("video_out.avi",
//                                        CV_FOURCC('P','I','M','1'),
//                                      fps,cvSize(frameW,frameH),isColor);
//}

void Robot::getGlutMask(int call)
{
	/* this function gets a mask image of the transformed space
	*   to help when projecting the visCvThresh image into world space.
	*    this function should be run in a loop so the updateGlutDisplay/OpenGL fully initializes the mask,
	*     and even still, the mask still isn't fully correct sometimes... */
	if (call==0) 
		ImageBufferManager::getInstance().setvisCvGlutMask(cvCreateImage(cvSize( ImageBufferManager::getInstance().getvisCvRaw()->width/2, ImageBufferManager::getInstance().getvisCvRaw()->height/2), IPL_DEPTH_8U, 3));
	cvSet( ImageBufferManager::getInstance().getvisCvRaw(), CV_RGB(255,255,255) );
	updateGlutDisplay();
	cvCopy(  ImageBufferManager::getInstance().getvisCvRawTransform(),  ImageBufferManager::getInstance().getvisCvRaw() );
	cvResize(  ImageBufferManager::getInstance().getvisCvRaw(),  ImageBufferManager::getInstance().getvisCvGlutMask(), CV_INTER_LINEAR );

	//if(call==0) cvNamedWindow("mask");
	//cvShowImage("mask",visCvGlutMask);
}


void Robot::update_vel_func()
{
	while(run_vel_thread)
	{
		double y,x;
		{
			boost::mutex::scoped_lock lock(velmutex);
			y = yvel;
			x = xvel;
		}
		if(osmcd->set_vel_vec(y, x))
		{
			std::cerr << "osmcd->set_vel_vec failed!" << std::endl;
		}
		
		//if(osmcd->updateVel_pd())
		//{
		//	std::cerr << "osmcd->updateVel_pd failed!" << std::endl;
		//}

		//usleep(5e4);
		usleep(1e5);
		//usleep(1e6);
	}

	{
		boost::mutex::scoped_lock lock(velmutex);
		osmcd->set_motors(0);
	}
}


void Robot::update_lidar_func()
{
	while(run_lidar_thread)
	{
		//if(!lidar->read())
		{
			boost::mutex::scoped_lock lock(lidarmutex);
		//	memcpy(this->coord, lidar->coord, sizeof(coord));
		//	lidar->findLinearRuns(lidar_linear_regions);
		//	lidar->getLongestRun(lidar_linear_regions, longest_linear_region);
		}
		//else
		//{
		//	std::cerr << "lidar read failed!" << std::endl;
		//}
		//do something with the lidar input
		usleep(1e5);
	}
}

