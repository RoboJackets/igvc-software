#include "mapgen.h"
#include "image_buffers.h"
#include <stdio.h>
#include "XmlConfiguration.h"
#include <stdlib.h>
#include "PointParamEstimator.h"
#include "Ransac.h"
#include <iostream>
#include <omp.h>

/* control what gets mapped into the world,
*   and whether to use the reactive sweeper lines in the worldmap */
#define PROCESS_MAP 1

/* Landmark Map Settings */
#define DO_LANDMARK_MAP 0
// below this, new world pts are considered the same, and updated
#define WORLDPT_PIXDIFF 1.5
// need this many counts to be considered a stable world pt
#define WORLDPT_MINGOODCOUNT 6
/* End Landmark Map Settings */




MapGen::MapGen()
{
}

MapGen::~MapGen()
{
	/* clean up */
	if (prev != NULL ) cvReleaseImage( &prev );
	if (points1 != NULL ) cvReleaseMat( &points1 );
	if (points2 != NULL ) cvReleaseMat( &points2 );
	if (status1 != NULL ) cvReleaseMat( &status1 );
	if (status2 != NULL ) cvReleaseMat( &status2 );
	if (matCamToCam != NULL ) cvReleaseMat( &matCamToCam );
	if (matCamToWorld != NULL ) cvReleaseMat( &matCamToWorld );
	if (worldmap != NULL ) cvReleaseImage( &worldmap );
}


/*
 * Main function
 */
int MapGen::genMap()
{
	/*
	 * this method should only be function calls
	 */


	/* get and rezise raw image to grayscale */
	{
		cvCvtColor(visCvRawTransform, visCvGreyBig, CV_BGR2GRAY);
		cvResize(visCvGreyBig, visCvGrey, CV_INTER_LINEAR);
	}


	/* testing image operations for better feature extraction */
	{
		//cvSmooth(visCvGrey,visCvGrey,CV_MEDIAN,3,0,0,0);
		//cvDilate( visCvGrey, visCvGrey, NULL, 3 );
		//cvErode( visCvGrey, visCvGrey, NULL, 1 );
		//cvSmooth(visCvGrey,visCvGrey,CV_MEDIAN,3,0,0,0);
		//cvSobel(visCvGrey,visCvGrey, 1, 1, 3);
		cvEqualizeHist(visCvGrey,visCvGrey);
	}


	/* get matching features from 2 greyscaled raw images.
	 *  don't continue if we don't have any */
	if ( !getFeatures() )
	{
		return 0;
	}

	/* process features found.
	 *  build matches vector and run RANSAC */
	if ( !processFeatures() )
	{
		return 0;
	}

#if DO_LANDMARK_MAP
	/* add found features to world feature list,
	 *  and plot/match all good features in the world frame */
	if ( !genLandmarkMap() )
	{
		return 0;
	}
#else
	/* generate a probability map of traversable area and obstacles
	*  based on the Thresh image.
	*   in the image:
	*     grey  = unknown
	*     white = traversable
	*     black = obstacle
	*/
	if ( !genProbabilityMap() )
	{
		return 0;
	}
#endif



	/* end genMap() - success */
	return 1;

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

		if (f)
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

		/* get sub-pixel accuracy */
		//// moved into cvcorrImages.cpp ////
		/*
		cvFindCornerSubPix(
		                prev,           //image
		                points1,        //corners
		                found,          //count
		                cvSize(5,5),    //window
		                cvSize(-1,-1),  //zero_zone (none)
		                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.05) //criteria
		                );*/

		if (found < 2)
		{
			// try again
			t--;
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
					1,      /* Use fundamental matrix to filter points? */
					0.5);   /* Threshold for (dist b/w) good points in filter (usually 0.5 or 1.0)*/

		//printf(" matching: %d \n",found);

		/* move current to previous */
		if (found && !f)
		{
			cvCopy(visCvGrey,prev);
		}

		/* local storage */
		int good=0;
		int x,y,a,b;
		int dx,dy;
		float avgdx=0;
		float avgdy=0;


		/* draw lines from prev to curr points in curr image */
		#pragma omp parallel for private(a,b,x,y,dx,dy) reduction(+:avgdx,avgdy)
		for (i=0; i<maxFeatures; i++)
		{
			/* only look at points in both images */
			if (cvGetReal1D(status2,i)&&cvGetReal1D(status1,i))
			{
				a=cvmGet(points1,0,i);
				b=cvmGet(points1,1,i);
				x=cvmGet(points2,0,i); //most recent
				y=cvmGet(points2,1,i); //most recent

				/* don't look in the horizon */
				if ( y > yFeatureThresh )
				{
					/* calculate slope and extend lines */
					dx = x-a;
					dy = y-b;
					avgdx += dx;
					avgdy += dy;
					good++;

					/* draw good matches */
					cvLine(visCvGrey, cvPoint( a-dx,b-dy ), cvPoint( x+dx,y+dy ), CV_RGB(0,0,0), 2, 8, 0);
				}

				/* draw all matches */
				//cvLine(visCvGrey, cvPoint( a,b ), cvPoint( x,y ), CV_RGB(0,0,0), 2, 8, 0);
			}
		}

		//printf("  using: %d \n",good);

		/* draw global direction vector in center (white) */
		if (good)
		{
			avgdx/=good;
			avgdy/=good;
			cvLine(visCvGrey,
				   cvPoint( imgHalfWidth-avgdx, imgHalfHeight-avgdy ),
				   cvPoint( imgHalfWidth+avgdx, imgHalfHeight+avgdy ),
				   CV_RGB(255,255,255), 3, 8, 0);
			//printf("   adx %.2f ady %.2f \n",avgdx,avgdy);
		}
		avgdx=abs(avgdx);
		avgdy=abs(avgdy);

		/* show image with points drawn */
		cvShowImage("curr",visCvGrey);


		/* check status *********/
		{
			if (avgdx<=0.05 && avgdy<=0.05)
			{
				return 0; // no motion
			}

			if ( ( avgdx > maxFeatureShift ) || ( avgdy > maxFeatureShift ) )
			{
				return 0; // bumpy/noisy motion
			}

			if ( good > 1 )
			{
				return 1; // we have enough frames/points
			}
			else
			{
				return 0; // need more points matching
			}
		}
	}


	return 0; // need more frames
}

int MapGen::processFeatures()
{

	/* storage */
	int i;
	double x,y,a,b;

	/* for RANSAC */
	std::vector<double> pointParameters;
	PointParamEstimator pointEstimator(0.3); //0.3; /* error percent */
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

			/* only add good points (close to bottom) */
			if ( y > yFeatureThresh )
			{
				/* gather feature matches pairs for RANSAC */
				foundpts[i] = cvPoint2D32f(a,b);
				foundpts[maxFeatures-1-i] = cvPoint2D32f(x,y);
				matchList.push_back( std::pair<CvPoint2D32f,CvPoint2D32f>( foundpts[i],foundpts[maxFeatures-1-i] ) );
			}

		}
	}

	//printMatchesVector(matchList);


	/*
	 * Run RANSAC on found features
	 */
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
		cvSetReal1D(matCamToCam, i, pointParameters.at(i) );
	}

	/* crude check for errors */
	if ( cvDet(matCamToCam) == 0 )
	{
		return 0; // bad matrix
	}

	//printCv33Matrix(matCamToCam);


	/* continuously mulitply each new c-c matrix to get new c-w matrix,
	 *  just not on the first run */
	static int do_map = 0;
	if (do_map)
	{
		cvInv(matCamToCam,matCamToCam); // convert cam-homography to map from curr to prev pts
		cvMatMul(matCamToWorld,matCamToCam,matCamToWorld); // map camera into world
	}
	else
	{
		do_map = 1;
	}

	//printCv33Matrix(matCamToWorld);

	/* continue */
	return 1;
}

void MapGen::init()
{
	/* load in params */
	LoadXMLSettings();

	/* init images and matricies */
	points1 = cvCreateMat(2,maxFeatures,CV_32F);
	points2 = cvCreateMat(2,maxFeatures,CV_32F);
	status1 = cvCreateMat(1,maxFeatures,CV_8SC1);
	status2 = cvCreateMat(1,maxFeatures,CV_8SC1);
	prev = cvCreateImage(cvGetSize(visCvGrey), 8, 1);
	cvZero(points1);
	cvZero(points2);
	cvZero(status1);
	cvZero(status2);

	/* misc */
	imgHalfHeight = visCvGrey->height/2;
	imgHalfWidth  = visCvGrey->width/2;

	/* amount of top of image to ignore */
	yFeatureThresh = visCvGrey->height/3 + 30 ;


	//==============================================================

	/* world map stuff */

	/* init images and matricies */
	int dx = 400; // center of
	int dy = 400; //   map
	worldmap = cvCreateImage( cvSize( dx*2,dy*2 ), IPL_DEPTH_8U, 3 );
	matCamToCam = cvCreateMat(3,3,CV_32F);
	matCamToWorld = cvCreateMat(3,3,CV_32F);
	cvZero(worldmap);
	cvZero(matCamToCam);
	cvZero(matCamToWorld);

	/* set up camera to world homography matrix */
	double k = .25;//0.3; // scale factor
	// k  0  dx
	// 0  k  dy
	// 0  0  1
	cvSetReal2D( matCamToWorld, 0, 0, k );
	cvSetReal2D( matCamToWorld, 0, 2, dx );
	cvSetReal2D( matCamToWorld, 1, 1, k );
	cvSetReal2D( matCamToWorld, 1, 2, dy );
	cvSetReal2D( matCamToWorld, 2, 2, 1 );
	//printCv33Matrix(matCamToWorld);

	//==============================================================

	/* probability map stuff */

	probmap = cvCreateImage( cvSize( dx*2,dy*2 ), IPL_DEPTH_8U, 1 );
	cvSet( probmap, CV_RGB(127,127,127), NULL);

	robotBaseAt = cvPoint(dx,dy);
	robotLookingAt = cvPoint(dx,dy-1);

	//==============================================================

	/* OpenMP support */

	// Get the number of processors in this system
	int iCPU = omp_get_num_procs();
	// Now set the number of threads
	omp_set_num_threads(iCPU);
	printf("omp num cpus %d \n", iCPU);

    //==============================================================

    /* process map stuff */
    cvNamedWindow("processmap");



}

void MapGen::LoadXMLSettings()
{
	/* load xml file */
	XmlConfiguration cfg("Config.xml");

	/* load settings */
	{
		maxFeatures   = cfg.getInt("maxFeatures");
		numFramesBack = cfg.getInt("numFramesBack");
		maxFeatureShift = cfg.getInt("maxFeatureShift");

		/* display windows */
		if (cfg.getInt("doMapping"))
		{
			cvNamedWindow("worldmap");
			cvNamedWindow("curr");
			cvNamedWindow("probmap");
		}

	}

	/* test */
	{
		if (maxFeatures==-1)
		{
			printf("ERROR: Mapping settings NOT loaded! Using DEFAULTS \n");
			{
				maxFeatures   = 40;
				numFramesBack = 3;
				maxFeatureShift = 9;

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

void MapGen::mapCamPointToWorldPoint(CvPoint2D32f& cam, CvPoint2D32f& world)
{
	/* maps a point in camera frame to world frame
	 *  via cam to world homography matrix */
	world.x = cvmGet(matCamToWorld,0,0) * cam.x +
			  cvmGet(matCamToWorld,0,1) * cam.y +
			  cvmGet(matCamToWorld,0,2);
	world.y = cvmGet(matCamToWorld,1,0) * cam.x +
			  cvmGet(matCamToWorld,1,1) * cam.y +
			  cvmGet(matCamToWorld,1,2);
}

void MapGen::mapCamPointToWorldPoint(double camx, double camy, double& worldx, double& worldy)
{
	/* maps a point in camera frame to world frame
	 *  via cam to world homography matrix */
	worldx =  cvmGet(matCamToWorld,0,0) * camx +
			  cvmGet(matCamToWorld,0,1) * camy +
			  cvmGet(matCamToWorld,0,2);
	worldy =  cvmGet(matCamToWorld,1,0) * camx +
			  cvmGet(matCamToWorld,1,1) * camy +
			  cvmGet(matCamToWorld,1,2);
}

void MapGen::addOrUpdateWorldPoint(CvPoint3D32f wpt)
{
	/* add first point */
	int n = worldPoints.size();
	if (n==0)
	{
		worldPoints.push_back(wpt);
		return;
	}

	/* pixel location difference threshold */
	double thresh = WORLDPT_PIXDIFF ;
	int i;
	CvPoint3D32f* curr;

	/* go through list of current world points to see if new point is
	 *  close enough to a point already seen, and increment its count */
	for (i=0; i<n; i++)
	{
		curr = &worldPoints.at(i);
		if ( (abs(curr->x-wpt.x)<thresh) && (abs(curr->y-wpt.y)<thresh) )
		{
			/* average point locations and update pt history */
			curr->x = (wpt.x+curr->x)/2;
			curr->y = (wpt.y+curr->y)/2;
			curr->z += 1;
			/* assume we hit the best match */
			break;
		}
	}

	/* if we didn't update anything, then this point is new */
	if (i==n)
	{
		worldPoints.push_back(wpt);
	}

}

int MapGen::genLandmarkMap()
{

	/* landmark processing */
	if (1)
	{
		double wx,wy;

		/* update all the found feature points into the world map point list */
		for (int i=0; i<maxFeatures; i++)
		{
			/* only matches */
			if (cvGetReal1D(status2,i)&&cvGetReal1D(status1,i))
			{
				/* get world coordinates */
				mapCamPointToWorldPoint( cvmGet(points2,0,i), cvmGet(points2,1,i), wx, wy );
				/* check point with world list */
				addOrUpdateWorldPoint( cvPoint3D32f( wx, wy, 0 ) );

				//cvCircle(worldmap, cvPoint( wx,wy ), 1, CV_RGB(rand()%255,rand()%255,rand()%255), 2, 8, 0);
			}
		}

		/* clear map */
		cvZero(worldmap);

		/* extract only features from world point list that have been seen a lot */
		std::vector< CvPoint3D32f >::iterator iter = worldPoints.begin();
		while (iter!=worldPoints.end())
		{
			/* get stable feature points */
			if (iter->z > WORLDPT_MINGOODCOUNT)
			{
				cvCircle(worldmap, cvPoint( iter->x, iter->y ), 1, CV_RGB(rand()%255,rand()%255,rand()%255), 2, 8, 0);
			}
			//printf("x=%.2f y=%.2f z=%.2f \n",iter->x,iter->y,iter->z);

			/* allow points to expire */
			iter->z -= 0.25;
			if (iter->z <= 0.5 ) worldPoints.erase(iter);
			else ++iter;
		}
	}


	/* draw a line from the center bottom of camera frame to center middle
	 *  to show the orientation of the robot in the world map */
	double x,y,a,b;
	mapCamPointToWorldPoint( imgHalfWidth, 0, a, b); // top center of camera frame
	mapCamPointToWorldPoint( imgHalfWidth, visCvGrey->height-1, x, y); // bottom center of camera frame
	/* update/avg current robot orientation */
	robotBaseAt = cvPoint( (robotBaseAt.x+x)/2, (robotBaseAt.y+y)/2 );
	robotLookingAt = cvPoint( (robotLookingAt.x+a)/2, (robotLookingAt.y+b)/2 );
	/* draw a line denoting heading orientaion */
	cvLine(worldmap, robotBaseAt, robotLookingAt,
		   //CV_RGB(rand()%255,rand()%255,rand()%255),
		   CV_RGB(255,255,255),
		   2, 8, 0);
	/* draw a circle denoting base orientaion */
	cvCircle(worldmap, robotBaseAt, 2,
			 //CV_RGB(rand()%255,rand()%255,rand()%255),
			 CV_RGB(255,255,255),
			 2, 8, 0);

	/* display world view image */
	cvShowImage("worldmap",worldmap);

	//cvWaitKey(0);

	return 1;
}

int MapGen::genProbabilityMap()
{
	/* in the probablity map image:
	 *  grey  = unknown
	 *  white = traversable
	 *  black = obstacle
	 */


	/* slowly move all probabilities toward unknown (127) */
	if (0)
	{
		double curr;
		double speed = 1;
		int i;

        #pragma omp parallel for private(curr)
		for (i=0; i<probmap->imageSize; i++)
		{
			curr = cvGetReal1D(probmap,i);
			if (curr==127) continue;
			else if (curr>127) cvSetReal1D(probmap,i,curr-speed);
			else /*if(curr<127)*/ cvSetReal1D(probmap,i,curr+speed);
			//else continue;
		}
	}


	/* down-scale the thresh img, and map it into into world,
	 *  while setting probabilities of obstacles and traversible area */
	int divby = 2;      // image size denominator
	int pad = 5;        // remove noise around img edges
	double badval = -2; // rate for obstacle probability
	double goodval = 7; // rate for travpath probability
	double setval;
	double wx,wy;
	int x,y;

    #pragma omp parallel for private(x,setval,wx,wy)
	for (y=pad; y<visCvThresh->height-divby-pad; y+=divby)
	{
		for (x=pad; x<visCvThresh->width-divby-pad; x+=divby )
		{
			// map pts
			mapCamPointToWorldPoint((double)x,(double)y,wx,wy);
			// get current val
			setval = cvGetReal2D(probmap,wy,wx);
			// and add/subtract based on thresh image
			#if PROCESS_MAP
			setval += (cvGetReal2D( /* visCvThresh */ visCvPath  ,y,x )==0)?badval:goodval;
			#else
            setval += (cvGetReal2D( visCvThresh /* visCvPath */ ,y,x )==0)?badval:goodval;
			#endif
			// cap results
			if (setval>255) setval = 255;
			else if (setval<0) setval = 0;
			// update probmap
			cvSetReal2D(probmap,wy,wx,setval);
		}
	}


	/* get robot orientation in world coordinates */
	double a,b;
	mapCamPointToWorldPoint(imgHalfWidth, 0, a, b); // top center of camera frame
	mapCamPointToWorldPoint(imgHalfWidth, visCvGrey->height-1, wx, wy); // bottom center of camera frame
	/* update/avg current robot orientation */
	robotBaseAt = cvPoint( (robotBaseAt.x+wx)/2, (robotBaseAt.y+wy)/2 );
	robotLookingAt = cvPoint( (robotLookingAt.x+a)/2, (robotLookingAt.y+b)/2 );
	//printf("bx=%d by=%d  lx=%d ly=%d \n",robotBaseAt.x,robotBaseAt.y,robotLookingAt.x,robotLookingAt.y);
	/* draw a circle denoting base orientaion */
//	cvCircle(probmap, CvPoint(robotBaseAt),
//			 1,
//			 CV_RGB(127,127,127),
//			 1, 8, 0);


	/* display */
	cvShowImage("probmap",probmap);
	//cvWaitKey(0);

	/* success */
	return 1;
}


// TESTING


#if PROCESS_MAP
#include "Graphics.h"
#endif

int MapGen::processMap()
{
#if PROCESS_MAP

    ////
    int nav_path__num = 9; //change 'vector' too
    int nav_path__center_path_id = nav_path__num/2;
    int nav_path__path_search_girth = 0; //<2
    int danger_per_barrel_pixel = 1; //=1
    int nav_path__danger_smoothing_radius = 3;
    int max_path_danger = 40;
    CvScalar dangerous_pixel_color = CV_RGB(255,0,0);
    IplImage* temp = cvCloneImage(probmap);
    IplImage* worldDebug = cvCreateImage(cvSize(probmap->width,probmap->height),8,3);
    cvCvtColor(temp, worldDebug, CV_GRAY2BGR);
    cvReleaseImage(&temp);
    double min_path_danger_value = 50;//20; // lower => be less afraid
    ////


	Graphics g_draw(worldDebug);
	int pathDanger[nav_path__num];
	int curPixelDanger = 0;
	int curPathDanger = 0;
	int weight = 0;

	/* Compute and draw all navigation paths we are considering (and do other actions) */
	{
    #pragma omp parallel for private(curPathDanger,curPixelDanger,weight)
		for (int pathID=0; pathID<nav_path__num; pathID++)
		{
			// Calculate path parameters
			Point2D<double> pathStart =navPath_start(/*pathID*/);
			Point2D<double> pathEnd = navPath_end(pathID);

			// Calculate the set of points in the path
			QVector< Point2D<int> > pathPoints;
			Graphics::calculatePointsInLine(
				(int) pathStart.x, (int) pathStart.y,
				(int) pathEnd.x, (int) pathEnd.y,			//
				&pathPoints);								//

			// Calculate the danger value for the path
			// along with the danger contributions of all
			// the pixels in the path
			curPathDanger = 0;
			uchar pixelDanger[pathPoints.count()];
			Point2D<int> curPoint;
			for (uint j=0, n2=pathPoints.count(); j<n2; j++)
			{
				// Calculate the danger contribution of the current path-pixel to the path-danger
				curPixelDanger = 0;
				//Point2D<int> curPoint;
				for (int delta=-nav_path__path_search_girth; delta<=nav_path__path_search_girth; delta++)
				{
					// (XXX: Consider *nearby* pixels as different fragments of the path-pixel)
					/*Point2D<int>*/ curPoint = pathPoints[j];
					curPoint.x += delta;

					// check path image (half size) for black=bad
					double val = cvGetReal1D(probmap,curPoint.y*probmap->width+curPoint.x);
					if (val <= min_path_danger_value)
					{
						// everything bad is a barrel
						curPixelDanger += danger_per_barrel_pixel;
					}
					else
					{
						// Nothing special: Probably grass
					}
				}

				// Compensate for considering *nearby* pixels
				curPixelDanger /= ((nav_path__path_search_girth*2) + 1);
				pixelDanger[j] = curPixelDanger;
				curPathDanger += curPixelDanger;
			}

			//==== weight outer path lines more scary than inner ==========//
			weight = abs(nav_path__center_path_id-pathID);
			curPathDanger += weight;

			// Clip high danger values to be no higher than max_path_danger
			if (curPathDanger > max_path_danger)
			{
				curPathDanger = max_path_danger;
			}
			pathDanger[pathID] = curPathDanger;

			// Draw the body of the path using a color that is determined from the path's danger value
			//g_draw.setColor(navPath_color(curPathDanger));
			g_draw.setColor( cvScalar(128,128, (curPathDanger / (double)max_path_danger) * 255) );
			g_draw.drawLine( (int) pathStart.x, (int) pathStart.y, (int) pathEnd.x, (int) pathEnd.y);

			g_draw.setColor(dangerous_pixel_color);
			// Hilight the "dangerous" pixels in the path
			// (that contributed to the path's total danger value)
			//Point2D<int> curPoint;
			for (uint j=0, n2=pathPoints.count(); j<n2; j++)
			{
				uchar curPixelDanger = pixelDanger[j];
				if (curPixelDanger != 0)
				{
					/*Point2D<int>*/ curPoint = pathPoints[j];

					// Color the pixel either thickly/thinly,
					// depending on how dangerous it is
					if (curPixelDanger > danger_per_barrel_pixel)  	// maximum danger
					{
						g_draw.drawRect_rational( curPoint.x, curPoint.y, 2, 2);
					}
					else
					{
						cvCircle( worldDebug, cvPoint(curPoint.x,curPoint.y), 1, CV_RGB(155,0,0), 1,8,0);
					}
				}
			}
		}
	}
	/*
	 * Apply smoothing to the path danger values so that paths
	 * that are *near* dangerous paths are also considered to
	 * be dangerous
	 */
	{
		int smoothedPathDangers[nav_path__num];

		// Copy first edge
		for (int curPath_id = 0; curPath_id < nav_path__danger_smoothing_radius; curPath_id++)
		{
			smoothedPathDangers[curPath_id] = pathDanger[curPath_id];
		}

		// Smooth interior
		int sumOfNearbyDangers = 0;
		int avgOfNearbyDangers = 0;
		for (int curPath_id = nav_path__danger_smoothing_radius;
				curPath_id < (nav_path__num - nav_path__danger_smoothing_radius + 1);
				curPath_id++)
		{
			sumOfNearbyDangers = 0;
			for (int delta = -nav_path__danger_smoothing_radius;
					delta < nav_path__danger_smoothing_radius;
					delta++)
			{
				sumOfNearbyDangers += pathDanger[curPath_id + delta];
			}
			avgOfNearbyDangers = sumOfNearbyDangers / (nav_path__danger_smoothing_radius*2 + 1);

			smoothedPathDangers[curPath_id] = avgOfNearbyDangers;
		}

		// Copy second edge
		for (int curPath_id = (nav_path__num - nav_path__danger_smoothing_radius);
				curPath_id < nav_path__num;
				curPath_id++)
		{
			smoothedPathDangers[curPath_id] = pathDanger[curPath_id];
		}

		// Transfer smoothed dangers back to primary danger buffer
		for (int curPath_id=0; curPath_id<nav_path__num; curPath_id++)
		{
			pathDanger[curPath_id] = smoothedPathDangers[curPath_id];
		}
	}

	/* Pick a path with a low danger-value to follow */
	{
		/*
		 * Find the path with the lowest danger value.
		 * If there are multiple such paths, pick the that is closest to the center
		 * (i.e., pick the path that points the most straight upward)
		 */
		int bestPath_id = 0;
		{
			int bestPath_danger = pathDanger[bestPath_id];
			int bestPath_distanceFromCenter = abs(nav_path__center_path_id - bestPath_id)+1;
			int curPath_danger;
			int curPath_distanceFromCenter;
			for (int curPath_id=0; curPath_id<nav_path__num; curPath_id++)
			{
				curPath_danger = pathDanger[curPath_id];
				curPath_distanceFromCenter = abs(nav_path__center_path_id - curPath_id);

				if (curPath_danger < bestPath_danger)
				{
					bestPath_danger = curPath_danger;
					bestPath_id = curPath_id;
					bestPath_distanceFromCenter = curPath_distanceFromCenter;
				}
				else if (curPath_danger == bestPath_danger)
				{
					if (curPath_distanceFromCenter < bestPath_distanceFromCenter)
					{
						bestPath_id = curPath_id;
						bestPath_distanceFromCenter = curPath_distanceFromCenter;
					}
				}
			}
		}

		/* Hilight the path that was picked */
		{
			//g_draw.setColor(navPath_color(pathDanger[bestPath_id]));
			g_draw.setColor( cvScalar( (pathDanger[bestPath_id] / (double)max_path_danger) * 255 ,128,128 ) );

			// Redraw the path, but much more thickly (in order to hilight it)

			Point2D<double> bestPath_start = navPath_start(/*bestPath_id*/);
			Point2D<double> bestPath_end = navPath_end(bestPath_id);


			for (int deltaX = -1; deltaX <= 1; deltaX++)
			{
				g_draw.drawLine(
					((int) bestPath_start.x) + deltaX, (int) bestPath_start.y,
					((int) bestPath_end.x) + deltaX, (int) bestPath_end.y);
			}

			/* Update goal */
            Point2D<int> goal; // TODO: send to robot.cpp to be averaged with vision output
            int scalex = 20;
            goal.x = (nav_path__center_path_id-bestPath_id)*scalex ;
            goal.y = max_path_danger - pathDanger[bestPath_id] ;

			//printf("goal(%d,%d) \n",goal.x,goal.y); // print in vision.cc
		}

		// DO IN MAIN:
		// 	drive motors
		// 	display motor output

	}

    cvShowImage("processmap",worldDebug);
    cvReleaseImage(&worldDebug);

    //cvWaitKey(0);
#endif

	return 1;
}

Point2D<double> MapGen::navPath_start(/*int pathID*/)
{
    Point2D<double> start;
    start.x = robotBaseAt.x;
	start.y = robotBaseAt.y;
    return start;
}
Point2D<double> MapGen::navPath_end(int pathID)
{
    return navPath_start(/*pathID*/) + navPath_vector(pathID);
}
Point2D<double> MapGen::navPath_vector(int pathID)
{

    int nav_path__center_path_id = 4; //nav_path__num/2;
    int nav_path__view_cone__spacing = 16;
    float nav_path__view_distance_multiplier = .60;//0.5;

	double x = (robotLookingAt.x - robotBaseAt.x) ;
	double y = (robotLookingAt.y - robotBaseAt.y) ; // positive y down
	//double y = robotBaseAt.y - robotLookingAt.y; // positive y down
	double rad = (atan(y/x)) ; // make up = 0 deg
	double radoff =( (nav_path__center_path_id-pathID)*nav_path__view_cone__spacing )*M_PI/180.0; // path selection
	rad += radoff; // rotate based on path
	if(x<0) rad += -M_PI ;
    //printf("  rad %.2f x %.2f y %.2f \n",rad,x,y);
    double radius = sqrtf(pow(x,2)+pow(y,2)); // dist formula
	return Point2D<double>(
			    radius*cos(rad),
			   radius*sin(rad) )   // computer y-axis is inverse of math y-axis
		   * nav_path__view_distance_multiplier;
}



