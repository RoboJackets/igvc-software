#include "mapgen.h"
#include "image_buffers.h"
#include <stdio.h>
#include "XmlConfiguration.h"


MapGen::MapGen()
{
	LoadXMLSettings();
	_init_ = 0;
}

MapGen::~MapGen()
{

	if (	eig_image != NULL )
	{
		cvReleaseImage( &eig_image );
	}

	if (	temp_image != NULL )
	{
		cvReleaseImage( &temp_image );
	}

}

/*
 * Main function
 */
void MapGen::genMap()
{
	/*
	 * this method should only be function calls
	 */

	/* initialize images */
	if (!_init_)
	{
		init();
	}

	/* get features from the greyscaled raw image */
	getFeatures();

}

void MapGen::getFeatures()
{

	cvCvtColor(visCvRaw, visCvGreyBig, CV_BGR2GRAY);
	cvResize(visCvGreyBig, visCvGrey, CV_INTER_LINEAR);

	CvPoint2D32f corners[maxFeatures];
	double quality = 0.01; //QUALITY; 0.01
	int win_size = 8; //WIN_SIZE; 16

	cvGoodFeaturesToTrack(
	    visCvGrey,   //const CvArr* image,
	    eig_image,   //CvArr* eig_image,
	    temp_image,  //CvArr* temp_image,
	    corners,     //CvPoint2D32f* corners,
	    &maxFeatures, //int* corner_count,
	    quality,     //double quality_level,
	    minFeatureDistance, //double min_distance,
	    NULL, //const CvArr* mask=NULL,
	    3,    //int block_size=3,
	    0,    //int use_harris=0,
	    0.04  //double k=0.04
	);

	//cvFindCornerSubPix( visCvGrey, corners, count,
	//                    cvSize(win_size,win_size), cvSize(-1,-1),
	//                    cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

	for (int i=0; i<maxFeatures; i++)
	{
		cvRectangle(visCvGrey, cvPoint(corners[i].x-win_size/2,corners[i].y-win_size/2),
		            cvPoint(corners[i].x+win_size/2,corners[i].y+win_size/2), CV_RGB(0,255,0));
		cvCircle(visCvGrey, cvPointFrom32f(corners[i]), 1, CV_RGB(255,0,0), -1, 8, 0);
	}

}

void MapGen::LoadXMLSettings()
{
	/* load xml file */
	XmlConfiguration cfg("Config.xml");

	/* load settings */
	{
		maxFeatures = cfg.getInt("maxFeatures");
		minFeatureDistance = cfg.getInt("minFeatureDistance");

	}

	/* test */
	{
		if (maxFeatures==-1 || minFeatureDistance==-1)
		{
			printf("ERROR: Mapping settings NOT loaded! Using DEFAULTS \n");
			{
				maxFeatures = 64;
				minFeatureDistance = 10;

			}
		}
		else
		{
			printf("Mapping settings loaded \n");
		}
		printf("values: maxFeatures %d  minFeatDist %d \n", maxFeatures, minFeatureDistance);
	}

}

void MapGen::init()
{

	eig_image = cvCreateImage( cvGetSize(visCvGrey), 32, 1 );
	temp_image = cvCreateImage( cvGetSize(visCvGrey), 32, 1 );


	_init_ = 1;
}


