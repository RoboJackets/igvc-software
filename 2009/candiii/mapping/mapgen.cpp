#include "mapgen.h"
#include "image_buffers.h"
#include <stdio.h>
#include "XmlConfiguration.h"

#define DEBUG 1

MapGen::MapGen()
{

}

MapGen::~MapGen()
{
    /* clean up */

    /*
    if (eig_image != NULL )
    {
        cvReleaseImage( &eig_image );
    }
    if (temp_image != NULL )
    {
        cvReleaseImage( &temp_image );
    }
    */

    if (prev != NULL )
    {
        cvReleaseImage( &prev );
    }
    if (points1 != NULL )
    {
        cvReleaseMat( &points1 );
    }
    if (points2 != NULL )
    {
        cvReleaseMat( &points2 );
    }
    if (status1 != NULL )
    {
        cvReleaseMat( &status1 );
    }
    if (status2 != NULL )
    {
        cvReleaseMat( &status2 );
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


    /* get and rezise raw image to grayscale */
    cvCvtColor(visCvRawTransform, visCvGreyBig, CV_BGR2GRAY);
    cvResize(visCvGreyBig, visCvGrey, CV_INTER_LINEAR);

    /* get features from the greyscaled raw image.
     *  return if we don't have any */
    if( !getFeatures() )
    {
        return;
    }

    /* process features */
    // TODO:


}

int MapGen::getFeatures()
{
    /* returns 1 when we have matching points and can proceed,
     *  or returns 0 otherwise. */

    int found;
    static int t=0;

    if (t==0)
    {
        // get first frame

        t++;

        cvCopy(visCvGrey, prev);

        icvCreateFeaturePoints(prev, points1, status1);

        return 0; // need more frames
    }
    else
    {
        // wait

        t++;
    }

    if (t==4)
    {
        // now match points

        t=0;

        found = icvFindCorrForGivenPoints(
                        prev,      /* Image 1 */
                        visCvGrey, /* Image 2 */
                        points1,
                        status1,
                        points2,
                        status2,
                        1,/*Use fundamental matrix to filter points */
                        0.5);/* Threshold for good points in filter */


        /* debug display */
        if(DEBUG)
        {
            printf("matching: %d \n",found);

            int x,y,a,b;

            // draw lines from prev to curr points in curr image
            for (int i=0; i<maxFeatures; i++)
            {
                if (cvGetReal1D(status2,i)&&cvGetReal1D(status1,i))
                {
                    a=cvmGet(points1,0,i);
                    b=cvmGet(points1,1,i);
                    x=cvmGet(points2,0,i);
                    y=cvmGet(points2,1,i);
                    cvLine(visCvGrey, cvPoint( x,y ), cvPoint( a,b ), CV_RGB(0,255,0), 3, 8, 0);
                }
            }

            // show images
            cvShowImage("curr",visCvGrey);
            //cvWaitKey(0);
        }

        if( found )
        {
            return 1; // we have enough frames/points
        }
        else
        {
            return 0; // no points matching
        }
    }

    return 0; // need more frames

}


/* DEPRICATED */
//void MapGen::getFeatures()
//{
//    /* get and rezise raw image to grayscale */
//    cvCvtColor(visCvRaw, visCvGreyBig, CV_BGR2GRAY);
//    cvResize(visCvGreyBig, visCvGrey, CV_INTER_LINEAR);
//    /* accuracy level */
//    double quality = 0.01; // 0.01;
//    /* extract feature points from image into 'corners' point array */
//    cvGoodFeaturesToTrack(
//        visCvGrey,   //const CvArr* image, source
//        eig_image,   //CvArr* eig_image, workspace for the algorithm.
//        temp_image,  //CvArr* temp_image, workspace for the algorithm.
//        features[1],     //CvPoint2D32f* corners, contains the feature points.
//        &maxFeatures, //int* corner_count, number feature points actually found
//        quality,     //double quality_level, specifies minimum quality of the features (based on the eigenvalues).
//        minFeatureDistance, //double min_distance, specifies the minimum Euclidean distance between features.
//        NULL, //const CvArr* mask=NULL, "NULL" means use the entire input image.
//        3,    //int block_size=3,
//        0,    //int use_harris=0,
//        0.04  //double k=0.04
//    );
//    /* This termination criteria tells the algorithm to stop when it has either done 20 iterations or when
//     * epsilon is better than .3.  You can play with these parameters for speed vs. accuracy but these values
//     * work pretty well in many situations. */
//    CvTermCriteria term_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );
//    /* search window half size */
//    int win_size = 8;
//    /* enhance point precision */
//    cvFindCornerSubPix(
//        visCvGrey,   // input image
//        features[1],     // initial corners in, refined corners out
//        maxFeatures, // number of corners
//        cvSize(win_size,win_size),  // half size of search window
//        cvSize(-1,-1), // dead region, -1 means none
//        term_criteria  // criteria for terminating iterative process
//    );
//    /* display found features */
//    for (int i=0; i<maxFeatures; i++)
//    {
//    	cvRectangle(visCvGrey, cvPoint(features[1][i].x-win_size/2,features[1][i].y-win_size/2),
//    	            cvPoint(features[1][i].x+win_size/2,features[1][i].y+win_size/2), CV_RGB(0,255,0));
//    	cvCircle(visCvGrey, cvPointFrom32f(features[1][i]), 1, CV_RGB(255,0,0), -1, 8, 0);
//    }
//}


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
    /* load in params */
    LoadXMLSettings();

    //features[0] = (CvPoint2D32f*)cvAlloc(maxFeatures*sizeof(features[0][0]));
    //features[1] = (CvPoint2D32f*)cvAlloc(maxFeatures*sizeof(features[0][0]));

    /* init temp images and matricies */
    points1 = cvCreateMat(2,maxFeatures,CV_32F);
    points2 = cvCreateMat(2,maxFeatures,CV_32F);
    status1 = cvCreateMat(1,maxFeatures,CV_8SC1);
    status2 = cvCreateMat(1,maxFeatures,CV_8SC1);
    prev = cvCreateImage(cvGetSize(visCvGrey), 8, 1);

    cvZero(points1);
    cvZero(points2);
    cvZero(status1);
    cvZero(status2);

    /* debug windows */
    if(DEBUG)
    {
        //cvNamedWindow("prev");
        cvNamedWindow("curr");
    }

}


