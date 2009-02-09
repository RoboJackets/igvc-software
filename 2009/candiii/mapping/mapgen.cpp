#include "mapgen.h"
#include "image_buffers.h"
#include <stdio.h>
#include "XmlConfiguration.h"
#include <stdlib.h>


#define NUMFRAMESBACK 5


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

    //cvSmooth(visCvGrey,visCvGrey,CV_MEDIAN,3,0,0,0);
    //cvDilate( visCvGrey, visCvGrey, NULL, 1 );
    //cvErode( visCvGrey, visCvGrey, NULL, 1 );

    /* get matching features from 2 greyscaled raw images.
     *  don't continue if we don't have any */
    if ( !getFeatures() )
    {
        return;
    }

    /* process features found */
    //processFeatures(); // WORK IN PROGRESS!


}

int MapGen::getFeatures()
{
    /* returns 1 when we have matching points and can proceed,
     *  or returns 0 otherwise. */

    int found, i;
    static int t=0;

    if (t==0)
    {
        t++;

        /* get first frame and its features */
        cvCopy(visCvGrey, prev);
        found = icvCreateFeaturePoints(prev, points1, status1);

        if (found==0)
        {
            t--; // try again
        }

        return 0; // need more frames
    }
    else
    {
        t++;
        /* wait NUMFRAMESBACK */
    }

    if (t==NUMFRAMESBACK)
    {
        t=0;

        /* now get current frame and match features with previous */
        found = icvFindCorrForGivenPoints(
                    prev,      /* Image 1 */
                    visCvGrey, /* Image 2 */
                    points1,
                    status1,
                    points2,
                    status2,
                    1,/*Use fundamental matrix to filter points */
                    0.1);/* Threshold for (dist b/w) good points in filter (usually 0.5)*/

        printf(" matching: %d \n",found);

        int x,y,a,b;

        /* draw lines from prev to curr points in curr image */
        for (i=0; i<maxFeatures; i++)
        {
            if (cvGetReal1D(status2,i)&&cvGetReal1D(status1,i))
            {
                a=cvmGet(points1,0,i);
                b=cvmGet(points1,1,i);
                x=cvmGet(points2,0,i);
                y=cvmGet(points2,1,i);
                cvLine(visCvGrey, cvPoint( x,y ), cvPoint( a,b ), CV_RGB(0,0,0), 2, 8, 0);
            }
        }

        /* show image with points drawn */
        cvShowImage("curr",visCvGrey);

        if ( found )
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

int MapGen::processFeatures()
{
    /* WORK IN PROGRESS! */
    /*  map each new camera frame into a world frame */

    static int map=0;
    int i;
    int x,y,a,b;
    int aff_count = 0;

    /* get 3 matching point pairs that are close to the robot (bottom 1/3 of image)
     *  and draw the chosen points */
    int min = visCvGrey->height * (2.0/3.0);
    for (i=maxFeatures-1; i>=0; i--)
    {
        if (cvGetReal1D(status2,i)&&cvGetReal1D(status1,i))
        {
            a=cvmGet(points1,0,i);
            b=cvmGet(points1,1,i);
            x=cvmGet(points2,0,i); //most recent
            y=cvmGet(points2,1,i); //most recent

            if ( (aff_count<3) && (y>min) /*&& (y<b)*/ )
            {

                cvCircle(visCvGrey, cvPoint( x,y ), 1, CV_RGB(255,255,255), 3, 8, 0);
                cvCircle(visCvGrey, cvPoint( a,b ), 1, CV_RGB(255,255,255), 3, 8, 0);
                pts2[aff_count].x = x;
                pts2[aff_count].y = y;
                pts1[aff_count].x = a;
                pts1[aff_count].y = b;
                aff_count++;
            }
        }
    }

    /* show image with points drawn */
    cvShowImage("curr",visCvGrey);

    CvMat* temp;
    temp = cvCreateMat(2,3,CV_32FC1);

    /* use the point pairs for getting the camera-camera affine transformation */
    if (aff_count>2)
    {
        cvGetAffineTransform(pts2,pts1,temp);
    }
    else
    {
        return 0; // need 3 point pairs
    }

    /* copy 2x3 matrix result into 3x3 matrix */
    cvZero(mat_CamToCam);
    for (i=0; i<3; i++)
    {
        cvmSet(mat_CamToCam, 0, i, cvmGet(temp, 0 ,i) );
        cvmSet(mat_CamToCam, 1, i, cvmGet(temp, 1 ,i) );
    }
    cvmSet(mat_CamToCam, 2, 2, 1);
    cvReleaseMat(&temp);

    /* crude check for errors */
    if( cvDet(mat_CamToCam) == 0 )
    {
        return 0; // bad matrix
    }

//    /* normalize affine transform, but not translation */
//    {
//        CvMat* mask;
//        mask = cvCreateMat(3,3,CV_8SC1);
//        cvZero(mask);
//        cvSetReal2D(mask, 0, 0, 1);
//        cvSetReal2D(mask, 0, 1, 1);
//        cvSetReal2D(mask, 1, 0, 1);
//        cvSetReal2D(mask, 1, 1, 1);
//        cvNormalize(mat_CamToCam,mat_CamToCam,1,0,CV_L2,mask);
//        cvReleaseMat(&mask);
//    }

    printMatrix(mat_CamToCam);

    /* continuously mulitply each new c-c matrix to get new c-w matrix,
     *  just not on the first run */
    if (map)
    {
        cvMatMul(mat_CamToWorld,mat_CamToCam,mat_CamToWorld);
    }
    map=1;


//    /* normalize world transform, but not translation */
//    {
//        CvMat* mask2;
//        mask2 = cvCreateMat(3,3,CV_8SC1);
//        cvZero(mask2);
//        cvSetReal2D(mask2, 0, 0, 1);
//        cvSetReal2D(mask2, 0, 1, 1);
//        cvSetReal2D(mask2, 1, 0, 1);
//        cvSetReal2D(mask2, 1, 1, 1);
//        cvNormalize(mat_CamToWorld,mat_CamToWorld, 1, 0,CV_L2,mask2);
//        cvReleaseMat(&mask2);
//    }

    printMatrix(mat_CamToWorld);

//    /* draw all the found feature points into the world map */
//        for (i=0; i<maxFeatures; i++)
//        {
//            if (cvGetReal1D(status2,i)&&cvGetReal1D(status1,i))
//            {
//                x= cvmGet(mat_CamToWorld,0,0) * cvmGet(points2,0,i) +
//                        cvmGet(mat_CamToWorld,0,1) * cvmGet(points2,1,i) +
//                        cvmGet(mat_CamToWorld,0,2);
//                y= cvmGet(mat_CamToWorld,1,1) * cvmGet(points2,1,i) +
//                        cvmGet(mat_CamToWorld,1,0) * cvmGet(points2,0,i) +
//                        cvmGet(mat_CamToWorld,1,2);
//
//                cvCircle(worldmap, cvPoint( x,y ), 1, CV_RGB(rand()%255,rand()%255,rand()%255), 2, 8, 0);
//            }
//        }

    /* draw a line from the center bottom of camera frame to center top
     *  to show the orientation of the robot in the world map */
    x= cvmGet(mat_CamToWorld,0,0) * visCvGrey->width/2 +
       cvmGet(mat_CamToWorld,0,1) * (visCvGrey->height-2) +
       cvmGet(mat_CamToWorld,0,2);
    y= cvmGet(mat_CamToWorld,1,0) * visCvGrey->width/2 +
       cvmGet(mat_CamToWorld,1,1) * (visCvGrey->height-2) +
       cvmGet(mat_CamToWorld,1,2);
    a= cvmGet(mat_CamToWorld,0,0) * visCvGrey->width/2 +
       cvmGet(mat_CamToWorld,0,1) * 1 +
       cvmGet(mat_CamToWorld,0,2);
    b= cvmGet(mat_CamToWorld,1,0) * visCvGrey->width/2 +
       cvmGet(mat_CamToWorld,1,1) * 1 +
       cvmGet(mat_CamToWorld,1,2);
    cvLine(worldmap, cvPoint( x,y ), cvPoint( a,b ), CV_RGB(rand()%255,rand()%255,rand()%255), 2, 8, 0);

    /* display world view image */
    cvShowImage("worldmap",worldmap);

   // cvWaitKey(0);

    return 0;
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
    cvNamedWindow("curr");

    //==============================================================

    /* world map stuff */

    int dx = 400;
    int dy = 400;
    worldmap = cvCreateImage( cvSize( dx*2,dy*2 ), 8, 3 );
    cvZero(worldmap);

    mat_CamToCam = cvCreateMat(3,3,CV_32F);
    mat_CamToWorld = cvCreateMat(3,3,CV_32F);

    double k = 0.1; // scale factor
    // k  0  dx
    // 0  k  dy
    // 0  0  1
    cvZero(mat_CamToWorld);
    cvSetReal2D( mat_CamToWorld, 0, 0, k );
    cvSetReal2D( mat_CamToWorld, 0, 2, dx );
    cvSetReal2D( mat_CamToWorld, 1, 1, k );
    cvSetReal2D( mat_CamToWorld, 1, 2, dy );
    cvSetReal2D( mat_CamToWorld, 2, 2, 1 );

    printMatrix(mat_CamToWorld);

    cvNamedWindow("worldmap");

    //==============================================================

}

void MapGen::printMatrix(CvMat* matrix)
{
    printf("\n");
    int row,col;
    for ( row = 0; row < 3; row++ )
    {
        for ( col = 0; col < 3; col++ )
        {
            printf(" %.2f ",(double)cvmGet( matrix, row, col ));
        }
        printf("\n");
    }
    printf("\n");
}
