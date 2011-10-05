#include "potentialfields.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>

int main()
{
	potentialfields pf;
	IplImage* img = cvLoadImage("../TestFiles/RoboJacketsRoboCup08.png", 0);
	bool* bitmap;
	
	// create a window
  	cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE); 
 	 cvMoveWindow("mainWin", 100, 100);

  	// show the image
  	cvShowImage("mainWin", img );

  	// wait for a key
 	cvWaitKey(0);

  	// release the image
  	cvReleaseImage(&img );

	// Will only run if TESTINGMODE is defined, meaning that the functions of class potentialfields have been made
	// public for testing purposes. 
	#ifdef TESTINGMODE
	
	#endif
	
	return 0;
}
