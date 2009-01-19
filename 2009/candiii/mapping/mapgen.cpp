#include "mapgen.h"
#include "image_buffers.h"
#include <stdio.h>
#include "XmlConfiguration.h"


MapGen::MapGen()
{
    /* load in params */
	LoadXMLSettings();
	/* flag for initializing temp images for features */
	_init_ = 0;
}

MapGen::~MapGen()
{
    /* clean up */
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
    /* get and rezise raw image to grayscale */
	cvCvtColor(visCvRaw, visCvGreyBig, CV_BGR2GRAY);
	cvResize(visCvGreyBig, visCvGrey, CV_INTER_LINEAR);

    /* storage for feature points */
	CvPoint2D32f corners[maxFeatures];
	/* accuracy level */
	double quality = 0.01; // 0.01;

    /* extract feature points from image into 'corners' point array */
	cvGoodFeaturesToTrack(
	    visCvGrey,   //const CvArr* image, source
	    eig_image,   //CvArr* eig_image, workspace for the algorithm.
	    temp_image,  //CvArr* temp_image, workspace for the algorithm.
	    corners,     //CvPoint2D32f* corners, contains the feature points.
	    &maxFeatures, //int* corner_count, number feature points actually found
	    quality,     //double quality_level, specifies the minimum quality of the features (based on the eigenvalues).
	    minFeatureDistance, //double min_distance, specifies the minimum Euclidean distance between features.
	    NULL, //const CvArr* mask=NULL, "NULL" means use the entire input image.
	    3,    //int block_size=3,
	    0,    //int use_harris=0,
	    0.04  //double k=0.04
	);

	/* This termination criteria tells the algorithm to stop when it has either done 20 iterations or when
	 * epsilon is better than .3.  You can play with these parameters for speed vs. accuracy but these values
	 * work pretty well in many situations. */
	CvTermCriteria term_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );
	/* search window half size */
	int win_size = 8;

    /* enhance point precision */
	cvFindCornerSubPix( visCvGrey, corners, maxFeatures,
	                    cvSize(win_size,win_size), cvSize(-1,-1),
	                    term_criteria
	                    );

    /* display found features */
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


