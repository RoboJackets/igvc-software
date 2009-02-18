#include "mapgen.h"
#include "image_buffers.h"
#include <stdio.h>
#include "XmlConfiguration.h"
#include <stdlib.h>

#include "PointParamEstimator.h"
#include "Ransac.h"
#include <iostream>


MapGen::MapGen()
{

}

MapGen::~MapGen()
{
    /* clean up */


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

    if (matCamToCam != NULL )
    {
        cvReleaseMat( &matCamToCam );
    }
    if (matCamToWorld != NULL )
    {
        cvReleaseMat( &matCamToWorld );
    }
    if (worldmap != NULL )
    {
        cvReleaseImage( &worldmap );
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


    /* testing image transforms for better feature extraction */
    {
        //cvPyrDown(visCvGreyBig,visCvGrey);
        //cvSmooth(visCvGrey,visCvGrey,CV_MEDIAN,3,0,0,0);
        //cvDilate( visCvGrey, visCvGrey, NULL, 3 );
        //cvErode( visCvGrey, visCvGrey, NULL, 1 );
        //cvSmooth(visCvGrey,visCvGrey,CV_MEDIAN,3,0,0,0);
        //cvSobel(visCvGrey,visCvGrey, 1, 1, 3);
        //cvLaplace( visCvGrey, visCvGrey, 3 );
    }


    /* get matching features from 2 greyscaled raw images.
     *  don't continue if we don't have any */
    if ( !getFeatures() )
    {
        return;
    }

    /* process features found */
    processFeatures(); // WORK IN PROGRESS!


}

int MapGen::getFeatures()
{
    /* returns 1 when we have matching points and can proceed,
     *  or returns 0 otherwise. */

    int found, i;
    static int t=0; // time state
    static int f=1; // initialy get raw frame

    if (t==0)
    {
        t++;

        if(f)
        {
            f=0;
            /* get raw first frame */
            cvCopy(visCvGrey, prev);
        }
        else
        {
            /* otherwise, use previous data when t==numFramesBack */
        }

        /* get its features */
        found = icvCreateFeaturePoints(prev, points1, status1);

        if (found==0)
        {
            t--; // try again
            f=1;
        }

        return 0; // need more frames
    }
    else
    {
        t++;
        /* wait NUMFRAMESBACK */
    }

    if (t==numFramesBack)
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
                    1,/* Use fundamental matrix to filter points? */
                    1);/* Threshold for (dist b/w) good points in filter (usually 0.5 or 1.0)*/

        printf(" matching: %d \n",found);

        /* move current to previous */
        if(found && !f)
        {
            cvCopy(visCvGrey,prev);
        }



        /* local storage */
        int x,y,a,b;
        int dx,dy;
        int avgdx=0;
        int avgdy=0;
        int good=0;

        /* draw lines from prev to curr points in curr image */
        for (i=0; i<maxFeatures; i++)
        {
            /* only look at points in both images */
            if (cvGetReal1D(status2,i)&&cvGetReal1D(status1,i))
            {
                a=cvmGet(points1,0,i);
                b=cvmGet(points1,1,i);
                x=cvmGet(points2,0,i);
                y=cvmGet(points2,1,i);

                /* don't look in the horizon */
                if(y>visCvGrey->height/3)
                {
                    /* calculate slope and extend lines */
                    dx = x-a;
                    dy = y-b;
                    avgdx += dx;
                    avgdy += dy;
                    good++;
                    //dx*=1;  // extend by X
                    //dy*=1;  // extend by X
                    cvLine(visCvGrey, cvPoint( a-dx,b-dy ), cvPoint( x+dx,y+dy ), CV_RGB(0,0,0), 2, 8, 0);
                }

                /* draw all matches */
                //cvLine(visCvGrey, cvPoint( a,b ), cvPoint( x,y ), CV_RGB(0,0,0), 2, 8, 0);
            }
        }

        /* draw global direction vector in center (white) */
        if(good)
        {
            avgdx/=good;
            avgdy/=good;
            avgdx*=3;   // extend by X
            avgdy*=3;   // extend by X
            cvLine(visCvGrey,
                cvPoint( visCvGrey->width/2-avgdx, visCvGrey->height/2-avgdy ),
                cvPoint( visCvGrey->width/2+avgdx, visCvGrey->height/2+avgdy ),
                CV_RGB(250,250,250), 3, 8, 0);
            //printf("%d, %d \n",avgdx,avgdy);
        }

        /* show image with points drawn */
        cvShowImage("curr",visCvGrey);


        /* check status *********/

        if(avgdx==0 || avgdy==0)
        {
            return 0; // crappy slopes
        }

        int maxd = 21;
        if( abs(avgdx) > maxd || abs(avgdy) > maxd )
        {
            return 0; // crappy slopes
        }


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

    /* storage */
    static int map=0;
    int i;
    double x,y,a,b;

    /* for affine transformation matrix calculation */
//    int aff_count = 0;
//    int min =  visCvGrey->height * (2.0/3.0);

    /* for RANSAC */
    std::vector<double> pointParameters;
    PointParamEstimator pointEstimator(0.4);
    matchList.clear();
    CvPoint2D32f foundpts[ maxFeatures*2 ];

    /* get matching point pairs */
    for (i=maxFeatures-1; i>=0; i--)
    {
        /* only look at points in both images */
        if (cvGetReal1D(status2,i)&&cvGetReal1D(status1,i))
        {
            a=cvmGet(points1,0,i);
            b=cvmGet(points1,1,i);
            x=cvmGet(points2,0,i); //most recent
            y=cvmGet(points2,1,i); //most recent

            /* only consider good points (close to bottom) */
            if(y > visCvGrey->height/3 )
            {
                /* gather feature matches pairs for RANSAC */
                foundpts[i] = cvPoint2D32f(a,b);
                foundpts[maxFeatures-1-i] = cvPoint2D32f(x,y);
                matchList.push_back( std::pair<CvPoint2D32f,CvPoint2D32f>( foundpts[i],foundpts[maxFeatures-1-i] ) );
            }

//            /* only consider good points */
//            if ( (aff_count<2) && (y*2>min) /*&& (y<b)*/ &&(y!=b)&&(x!=a) )
//            {
//                /* gather points for affine calculation */
//                cvCircle(visCvGrey, cvPoint( x,y ), 1, CV_RGB(255,255,255), 3, 8, 0);
//                cvCircle(visCvGrey, cvPoint( a,b ), 1, CV_RGB(255,255,255), 3, 8, 0);
//                pts2[aff_count].x = x;
//                pts2[aff_count].y = y;
//                pts1[aff_count].x = a;
//                pts1[aff_count].y = b;
//                aff_count++;
//            }

        }
    }

    /* show image */
//    cvShowImage("curr",visCvGrey);


//    CvMat* temp;
//    temp = cvCreateMat(2,3,CV_32FC1);
//
//    /* use the point pairs for getting the camera-camera affine transformation */
//    if (aff_count>2)
//    {
//        cvGetAffineTransform(pts2,pts1,temp);
//    }
//    else
//    {
//        return 0; // need 3 point pairs
//    }
//
//    /* copy 2x3 matrix result into 3x3 matrix */
//    cvZero(matCamToCam);
//    for (i=0; i<3; i++)
//    {
//        cvmSet(matCamToCam, 0, i, cvmGet(temp, 0 ,i) );
//        cvmSet(matCamToCam, 1, i, cvmGet(temp, 1 ,i) );
//    }
//    cvmSet(matCamToCam, 2, 2, 1);
//    cvReleaseMat(&temp);
//
//    if(aff_count>1)
//    {
//        if(! computeHomography(pts1,pts2,matCamToCam) )
//            return 0;
//    }
//    else
//    {
//        return 0;
//    }


//    printMatchesVector(matchList);

    /* Run RANSAC on found features */
    //double usedData =
    Ransac<CvPoint2D32f,double>::compute(
    					pointParameters,    // output cam-cam homography matrix
                    	&pointEstimator,    // templeted class for comparing point matches
	                    matchList,          // vector of point match pairs
    	                2                   // number of matches required to compute homography (must be 2)
    	                );

    /* put RANSAC homography matrix ouput into a CvMat */
    cvZero(matCamToCam);
    for (i=0; i<9; i++)
    {
        if( pointParameters.at(i) > 1000 ) return 0; // check for an incorrect matrix
        cvSetReal1D(matCamToCam, i, pointParameters.at(i) );
    }


    /* crude check for errors */
    if( cvDet(matCamToCam) == 0 )
    {
        return 0; // bad matrix
    }

//    /* normalize camera transform, but not translation */
//    {
//        CvMat* mask;
//        mask = cvCreateMat(3,3,CV_8SC1);
//        cvZero(mask);
//        cvSetReal2D(mask2, 0, 0, 1); cvSetReal2D(mask2, 0, 1, 1);
//        cvSetReal2D(mask2, 1, 0, 1); cvSetReal2D(mask2, 1, 1, 1);
//        cvNormalize(matCamToCam,matCamToCam,1,0,CV_L2,mask);
//        cvReleaseMat(&mask);
//    }

    printCv33Matrix(matCamToCam);

    /* continuously mulitply each new c-c matrix to get new c-w matrix,
     *  just not on the first run */
    if (map)
    {
        cvMatMul(matCamToWorld,matCamToCam,matCamToWorld);
    }
    map=1;


//    /* normalize world transform, but not translation */
//    {
//        CvMat* mask2;
//        mask2 = cvCreateMat(3,3,CV_8SC1);
//        cvZero(mask2);
//        cvSetReal2D(mask2, 0, 0, 1); cvSetReal2D(mask2, 0, 1, 1);
//        cvSetReal2D(mask2, 1, 0, 1); cvSetReal2D(mask2, 1, 1, 1);
//        cvNormalize(matCamToWorld,matCamToWorld, 1, 0,CV_L2,mask2);
//        cvReleaseMat(&mask2);
//    }

    printCv33Matrix(matCamToWorld);

//    /* draw all the found feature points into the world map */
//        for (i=0; i<maxFeatures; i++)
//        {
//            if (cvGetReal1D(status2,i)&&cvGetReal1D(status1,i))
//            {
//                x= cvmGet(matCamToWorld,0,0) * cvmGet(points2,0,i) +
//                        cvmGet(matCamToWorld,0,1) * cvmGet(points2,1,i) +
//                        cvmGet(matCamToWorld,0,2);
//                y= cvmGet(matCamToWorld,1,1) * cvmGet(points2,1,i) +
//                        cvmGet(matCamToWorld,1,0) * cvmGet(points2,0,i) +
//                        cvmGet(matCamToWorld,1,2);
//
//                cvCircle(worldmap, cvPoint( x,y ), 1, CV_RGB(rand()%255,rand()%255,rand()%255), 2, 8, 0);
//            }
//        }

    /* draw a line from the center bottom of camera frame to center top
     *  to show the orientation of the robot in the world map */
    x= cvmGet(matCamToWorld,0,0) * visCvGrey->width/2 +
       cvmGet(matCamToWorld,0,1) * (visCvGrey->height-2) +
       cvmGet(matCamToWorld,0,2);
    y= cvmGet(matCamToWorld,1,0) * visCvGrey->width/2 +
       cvmGet(matCamToWorld,1,1) * (visCvGrey->height-2) +
       cvmGet(matCamToWorld,1,2);
    a= cvmGet(matCamToWorld,0,0) * visCvGrey->width/2 +
       cvmGet(matCamToWorld,0,1) * 1 +
       cvmGet(matCamToWorld,0,2);
    b= cvmGet(matCamToWorld,1,0) * visCvGrey->width/2 +
       cvmGet(matCamToWorld,1,1) * 1 +
       cvmGet(matCamToWorld,1,2);
    cvLine(worldmap, cvPoint( x,y ), cvPoint( a,b ), CV_RGB(rand()%255,rand()%255,rand()%255), 2, 8, 0);
    /* draw a circle denoting orientaion */
    cvCircle(worldmap, cvPoint( x,y ), 2, CV_RGB(rand()%255,rand()%255,rand()%255), 2, 8, 0);

    /* display world view image */
    cvShowImage("worldmap",worldmap);

    //cvWaitKey(0);

    return 1;
}

void MapGen::init()
{
    /* load in params */
    LoadXMLSettings();

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

    matCamToCam = cvCreateMat(3,3,CV_32F);
    matCamToWorld = cvCreateMat(3,3,CV_32F);

    double k = 0.1; // scale factor
    // k  0  dx
    // 0  k  dy
    // 0  0  1
    cvZero(matCamToWorld);
    cvSetReal2D( matCamToWorld, 0, 0, k );
    cvSetReal2D( matCamToWorld, 0, 2, dx );
    cvSetReal2D( matCamToWorld, 1, 1, k );
    cvSetReal2D( matCamToWorld, 1, 2, dy );
    cvSetReal2D( matCamToWorld, 2, 2, 1 );

    printCv33Matrix(matCamToWorld);

    cvNamedWindow("worldmap");

    //==============================================================

}

void MapGen::LoadXMLSettings()
{
    /* load xml file */
    XmlConfiguration cfg("Config.xml");

    /* load settings */
    {
        maxFeatures = cfg.getInt("maxFeatures");
        numFramesBack = cfg.getInt("numFramesBack");

    }

    /* test */
    {
        if (maxFeatures==-1)
        {
            printf("ERROR: Mapping settings NOT loaded! Using DEFAULTS \n");
            {
                maxFeatures = 64;
                numFramesBack = 4;

            }
        }
        else
        {
            printf("Mapping settings loaded \n");
        }
        printf("values: maxFeatures %d  \n", maxFeatures);
    }

}

void MapGen::printCv33Matrix(CvMat* matrix)
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


//int MapGen::computeHomography(CvPoint2D32f* p1, CvPoint2D32f* p2, CvMat* h)
//{
//    /*
//        | 1 b e |   | x1 |   | x2 |
//        | c 1 f | * | y1 | = | y2 |
//        | 0 0 1 |   | 1  |   | 1  |
//    */
//
//    double x1,y1,x2,y2,x3,y3,x4,y4,b,c,e,f;
//
//    x1 = p1[0].x;
//    y1 = p1[0].y;
//    x2 = p2[0].x;
//    y2 = p2[0].y;
//
//    x3 = p1[1].x;
//    y3 = p1[1].y;
//    x4 = p2[1].x;
//    y4 = p2[1].y;
//
//    b = ( x2-x4-x1+x3 ) / ( y1-y3 ) ;
//    c = ( y2-y4-y1+y3 ) / ( x1-x3 ) ;
//
//    e = ( x2+x4-x1-x3-b*(y1+y3) ) / 2 ;
//    f = ( y2+y4-y1-y3-c*(x1+x3) ) / 2 ;
//
//    cvZero(h);
//
//    cvmSet(h,0,0, 1 ); cvmSet(h,0,1, b ); cvmSet(h,0,2, e );
//    cvmSet(h,1,0, c ); cvmSet(h,1,1, 1 ); cvmSet(h,1,2, f );
//    cvmSet(h,2,0, 0 ); cvmSet(h,2,1, 0 ); cvmSet(h,2,2, 1 );
//
//    printf(" (%.2f,%.2f)-(%.2f %.2f)  (%.2f,%.2f)-(%.2f %.2f)  \n" , x1,y1,x2,y2,x3,y3,x4,y4 );
//    printf(" %.2f %.2f %.2f %.2f  \n" , b,c,e,f );
//
//    if( y1==y3 || x1==x3 ) return 0;
//    //else if( abs(e)>35 || abs(f)>25 ) return 0;
//    else return 1;
//
//}






