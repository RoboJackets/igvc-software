#include "Robot.h"
#include "vision.h"
#include "main.h"
#include "image_buffers.h"
#include "vision_processing.h"
#include "PjMat.h"

#include <stdlib.h>
#include <GL/glut.h>

// flag for saving video - global because of glut use
int saveRawVideo;

/* GLUT callbacks and functions ***********************/
pthread_t Robot::robotThread; // for pthread_create
Robot* glRobot; // for glut
static GLuint cameraImageTextureID; // for glut
int glutwindow; // for glut
void* robot_thread_caller(void* arg) {
    saveRawVideo=0; // don't save video yet
    glRobot = (Robot*)arg; // assign pointer to static robot object for glut to use
    static_cast<Robot*>(arg)->Go(); // start the robot
    return NULL;
}
void robot_process_function_caller(void) {
    glRobot->processFunc(); // for glutMainLoop
}
void idleFunc(void) { // for refresheing glut window
    glutPostRedisplay();
}
void keyboardFunc(unsigned char key, int x, int y) { // handles keyboard button presses
    switch (key) {
    case 'd':
        printf("die! \n");
        glRobot->destroy(); // kill the robot;
        break;
    case 's':
    	saveRawVideo = 1-saveRawVideo;
    	printf("video file \n");
    	break;
    default:
        break;
    }
}
/*******************************************/


//Robot::Robot() {
//    /* connect to the camera */
//    connectToCamera();
//}

Robot::Robot(const char* filename) {

    // only the CVcam can load video...
#if  USE_FIREWIRE_CAMERA
    filename=NULL;
#endif

    if (filename==NULL)
        /* connect to the camera */
        connectToCamera();
    else
        /* load a video */
        camera.connect(0, filename);
        

    
}

Robot::~Robot() {
	cvReleaseVideoWriter(&cvVideoWriter);
    destroy();
}

void Robot::destroy() {
    /* clean up */
    glutDestroyWindow(glutwindow);
    cvDestroyAllWindows();
    exit(0);
}

int Robot::init() {

	/* setup image selection bar */
	trackbarVal = 1; // currently selected image to view (see ConvertAllImageViews() in vision.cc)
	int numberOfViews = 10; // important!!!
	cvCreateTrackbar("bar","display",&trackbarVal,numberOfViews,trackbarHandler);
	
    /* try to grab a frame to get image size */
    if(!camera.GrabCvImage())
    	return 0; // fail
    
    /* configure opencv display window */
    cvResizeWindow( "display", visCvRaw->width, visCvRaw->height );
    cvMoveWindow( "display", 10, 10 ); // position on screen
    
    /* init all CV images here */
    {
    	/* 3 plane images */
	    visCvDebug	= cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 3);
	    visCvHSV 		= cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 3);
		visCvHSVSmall = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 3);
	    
	    /* 1 plane images */
	    
	    //visCvRedChannel   	= cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 1);
	    //visCvGreenChannel	= cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 1);
	    //visCvBlueChannel		= cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 1);
	    
	    /* Note: these are 320x240 & 1 plane images! */
	    visCvHue 				= cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
	    visCvSaturation	= cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
	    visCvGrey 			= cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
	    visCvThresh = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
	    visCvPath		= cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
    }

    /* load in vision settings */
    LoadVisionXML(); // in vision.cc

    /* set cleanup on exit */
    atexit(Robot::destroy);
    
    /* init video writer */
    createVideoWriter();

    /* success */
    return 1;
}

void Robot::Go() {
    /*
     * This function initializes and stars the robot
     */

    /* Quit if there is no camera */
    if (!camera.isValid())
        return;

    /* Try to grab a frame to get image size */
    if(!camera.GrabCvImage())
    	return;

    /* Setup video card processing */
    initGlut();

    /* Quit if we can't initialize properly */
    if (!init())
        return;
        
    /* Init default view (debug=1) */
    trackbarHandler( 1 );

    /*
     * Robot Loop!
     */
    glutMainLoop(); // runs processFunc() 

}

void Robot::processFunc() {
    /*
     * This function should be just simple function calls.
     */

    /* Get raw image */;
    camera.GrabCvImage();
    
    
    /* Shove raw image into graphics card for some processing on the card */
    updateGlutDisplay();

    /* Get sensor information */


    /* Perform vision processing. */
    visProcessFrame(heading);

    /* Make decision */


    /* Update displays */
	ConvertAllImageViews(); // display views based on trackbar position

    /* Drive Robot via motor commands (GO!) */


	
	/* Save raw image last */
	if(saveRawVideo){
		cvWriteFrame(cvVideoWriter,visCvRaw);
	}


}

void Robot::startRobotThread(void* obj) {
    sleep(1);
    //pthread_create(&robotThread, NULL, robot_thread_caller, obj);
    robot_thread_caller(obj);
}

void Robot::connectToCamera() {
    if (!camera.connect()) {
        printf("Camera connect failure \n");
        printf("Try using sudo... \n");
        exit(-1);
    } else {
        camera.loadSettings();
        printf("Camera settings loaded \n");
    }
}

void Robot::initGlut() {

	// dummy args
    int argc = 1;
    char** argv;

    glutInit(&argc, argv);
    glutInitWindowSize(visCvRaw->width, visCvRaw->height);
    glutInitWindowPosition(800, 480); // position on screen
    glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
    glutwindow = glutCreateWindow("Transform");
    glutDisplayFunc(robot_process_function_caller); // the function glutMainLoop() runs
    glutIdleFunc(idleFunc);
    glutKeyboardFunc(keyboardFunc);
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    glGenTextures(1, &cameraImageTextureID);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraImageTextureID);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

}

void Robot::updateGlutDisplay() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    
	    //glMatrixMode(GL_PROJECTION);
	    //glLoadIdentity();
	    //gluOrtho2D(	0.0, (GLdouble)visCvRaw->width,	0.0, (GLdouble)visCvRaw->height);
	    //glMatrixMode(GL_MODELVIEW);
	    //glLoadIdentity();
	    //glMatrixMode(GL_MODELVIEW);
	    
	    /* * * transform * * */
		glLoadIdentity ();
		glOrtho (-1.0, 1.0, -1.0, 1.0, -1.0, 1.0); // sets up basic scale for input for you to draw on
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity ();					
		/* * * * * * * * * * */
	
	    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraImageTextureID);
	    /* put data in card */
	    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, visCvRaw->width, visCvRaw->height, 0, GL_BGR, GL_UNSIGNED_BYTE, visCvRaw->imageData);
	    
	    /* * * transform * * */
	    setPjMat();
	    /* * * * * * * * * * */
	    
	    glBegin(GL_QUADS);
	    
	    	// default perspective (upside down)
		    //glTexCoord2i(0, 0); 								glVertex2i(0, 0);
		    //glTexCoord2i(visCvRaw->width, 0);					glVertex2i(visCvRaw->width, 0);
		    //glTexCoord2i(visCvRaw->width, visCvRaw->height);	glVertex2i(visCvRaw->width, visCvRaw->height);
		    //glTexCoord2i(0, visCvRaw->height);	  			glVertex2i(0, visCvRaw->height);
			// corrected perspective (normal)
		    //glTexCoord2i(0, 					visCvRaw->height);	glVertex2i(0, 				0);
		    //glTexCoord2i(visCvRaw->width, 	visCvRaw->height);	glVertex2i(visCvRaw->width, 0);
		    //glTexCoord2i(visCvRaw->width, 	0);					glVertex2i(visCvRaw->width, visCvRaw->height);
		    //glTexCoord2i(0, 					0); 				glVertex2i(0, 				visCvRaw->height);

			/* * * transform * * */
			glTexCoord2i(0, 				0);					glVertex3f(-1,	-1,	0);
		    glTexCoord2i(visCvRaw->width, 	0);					glVertex3f( 1,	-1,	0);
		    glTexCoord2i(visCvRaw->width, 	visCvRaw->height);	glVertex3f( 1,   1,	0);
		    glTexCoord2i(0, 				visCvRaw->height); 	glVertex3f(-1,	 1,	0);
		    /* * * * * * * * * * */
		    
	    glEnd();
	    
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
  
    /* get data from card */
    //glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, visCvRaw->imageData);
    glReadPixels(	0				,	//GLint x,
			     	0				,	//GLint y,
			     	visCvRaw->width	,	//GLsizei width,
			     	visCvRaw->height,	//GLsizei height,
			     	GL_BGR			,	//GLenum format,
			     	GL_UNSIGNED_BYTE,	//GLenum type,
			     	visCvRaw->imageData //Image	
			     	);
    
    // double buffering
    glutSwapBuffers();

}

void Robot::createVideoWriter()
{
	cvVideoWriter = 0;
	int isColor = 1;
	int fps     = 10;  // or 30
	int frameW  = visCvRaw->width; // 
	int frameH  = visCvRaw->height; // 
	
	/*	Other possible codec codes:
		CV_FOURCC('P','I','M','1')    = MPEG-1 codec
		CV_FOURCC('M','J','P','G')    = motion-jpeg codec (does not work well)
		CV_FOURCC('M', 'P', '4', '2') = MPEG-4.2 codec
		CV_FOURCC('D', 'I', 'V', '3') = MPEG-4.3 codec
		CV_FOURCC('D', 'I', 'V', 'X') = MPEG-4 codec
		CV_FOURCC('U', '2', '6', '3') = H263 codec
		CV_FOURCC('I', '2', '6', '3') = H263I codec
		CV_FOURCC('F', 'L', 'V', '1') = FLV1 codec
	*/
	cvVideoWriter=cvCreateVideoWriter("video_out.avi",CV_FOURCC('P','I','M','1'),
	                           fps,cvSize(frameW,frameH),isColor);
                           
}


