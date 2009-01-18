#include "mapgen.h"

#include <cv.h>
#include <highgui.h>
#include "Point2D.h"
#include "image_buffers.h"


MapGen::MapGen()
{
	//ctor
}

MapGen::~MapGen()
{
	//dtor
}

/*
 * Main function
 */
void MapGen::genMap()
{
	/* this method should only be function calls */

	/* get features from the greyscaled raw image */
	getFeatures();

}

void MapGen::getFeatures()
{

	cvCvtColor(visCvRaw, visCvGreyBig, CV_BGR2GRAY);
	cvResize(visCvGreyBig, visCvGrey, CV_INTER_LINEAR);

	IplImage* eig = cvCreateImage( cvGetSize(visCvGrey), 32, 1 );
	IplImage* temp = cvCreateImage( cvGetSize(visCvGrey), 32, 1 );

#define MAX_COUNT 32
	int count = MAX_COUNT;
	CvPoint2D32f corners[MAX_COUNT];
	double quality = 0.01; //QUALITY; 0.01
	double min_distance = 20; //MIN_DISTANCE; 10
	int win_size = 8; //WIN_SIZE; 16
#undef MAX_COUNT

	cvGoodFeaturesToTrack( visCvGrey, eig, temp, corners, &count,
	                       quality, min_distance, NULL, 3, 0, 0.04 );

	cvFindCornerSubPix( visCvGrey, corners, count,
	                    cvSize(win_size,win_size), cvSize(-1,-1),
	                    cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

	cvReleaseImage( &eig );
	cvReleaseImage( &temp );


	for (int i=0; i<count; i++)
	{
		cvRectangle(visCvGrey, cvPoint(corners[i].x-win_size/2,corners[i].y-win_size/2),
		            cvPoint(corners[i].x+win_size/2,corners[i].y+win_size/2), CV_RGB(0,255,0));
		cvCircle(visCvGrey, cvPointFrom32f(corners[i]), 1, CV_RGB(255,0,0), -1, 8, 0);
	}

}
