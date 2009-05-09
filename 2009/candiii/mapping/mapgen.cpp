#include "mapgen.h"
#include "image_buffers.h"
#include <stdio.h>
#include "XmlConfiguration.h"
#include <stdlib.h>
#include "PointParamEstimator.h"
#include "Ransac.h"
#include <iostream>
#include <omp.h>


/* Percent of new robot world position to use */
#define K_  0.70

/* Shift robot world position from currently calcualted position */
#define BASE_OFFSET -5  // smaller is further back

/* Use visCvPath (more black) or visCvThresh (more correct) to plot into worldmap */
#define USE_PATH_IMG  0

/* Avg dx dy of features must be grater than this */
#define MIN_FEATURE_SHIFT 0.05


/* Landmark Map Settings */ ////////////// (not used)
#define DO_LANDMARK_MAP 0
// below this, new world pts are considered the same, and updated
#define WORLDPT_PIXDIFF 1.5
// need this many counts to be considered a stable world pt
#define WORLDPT_MINGOODCOUNT 6
/* End Landmark Map Settings */ /////////


/* general absolute value */
#define fabs(x) ((x<0)?(-x):(x))



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
	if (probmap != NULL ) cvReleaseImage( &probmap );
	if (worldDebug != NULL ) cvReleaseImage( &worldDebug );
}


/*
 * Main function
 */
int MapGen::genMap()
{
	/*
	 * this method should only be function calls
	 */


	/* get raw image to grayscale */
	{
		cvCvtColor(visCvRawTransformSmall, visCvGrey, CV_BGR2GRAY);
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
	//static int _t=0; // time state
	//static int _f=1; // initialy get raw frame

	if (_t==0)
	{
		_t++;

		if (_f)
		{
			_f=0;
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
			_t--;
			_f=1;
		}

		return 0; // need more frames
	}
	else
	{
		_t++;
		/* wait NUMFRAMESBACK */
	}

	if (_t==numFramesBack)
	{
		_t=0;

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
		if (found && !_f)
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
			//printf("   adx %.4f ady %.4f \n",avgdx,avgdy);
		}
		avgdx=fabs(avgdx);
		avgdy=fabs(avgdy);
		//printf("   adx %.4f ady %.4f \n",avgdx,avgdy);

		/* show image with points drawn */
		cvShowImage("curr",visCvGrey);


		/* check status *********/
		{
			if (avgdx<=MIN_FEATURE_SHIFT && avgdy<=MIN_FEATURE_SHIFT)
			{
				printf(" no motion \n");
				return 0; // no motion
			}

			if ( ( avgdx > maxFeatureShift ) || ( avgdy > maxFeatureShift ) )
			{
				printf(" feature shift too big \n");
				return 0; // bumpy/noisy motion
			}

			if ( good > 2 )
			{
				return 1; // we have enough frames/points
			}
			else
			{
				printf(" need more matches \n");
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
	float x,y,a,b;

	/* for RANSAC */
	std::vector<float> pointParameters;
	PointParamEstimator pointEstimator(0.25); //0.3; /* error percent */
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
	Ransac<CvPoint2D32f,float>::compute(
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
	//static int _do_map = 0;
	if (_do_map)
	{
		cvInv(matCamToCam,matCamToCam); // convert cam-homography to map from curr to prev pts
		cvMatMul(matCamToWorld,matCamToCam,matCamToWorld); // map camera into world
	}
	else
	{
		_do_map = 1;
	}

	//printCv33Matrix(matCamToWorld);

	/* continue */
	return 1;
}

void MapGen::init()
{
	/* load in params first */
	LoadMappingXMLSettings();

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
	yFeatureThresh = visCvGrey->height/2 -15 ; // higher => skip more of top

	//==============================================================

	/* world map stuff */

	/* init images and matricies */
	int dx = 450; // center of
	int dy = 420; //   map
	worldmap = cvCreateImage( cvSize( dx*2,dy*2 ), IPL_DEPTH_8U, 3 );
	matCamToCam = cvCreateMat(3,3,CV_32F);
	matCamToWorld = cvCreateMat(3,3,CV_32F);
	cvZero(worldmap);
	cvZero(matCamToCam);
	cvZero(matCamToWorld);

	/* set up camera to world homography matrix */
	float k = 0.25; //0.3; // scale factor
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

	/* previously static variables */
	_t = 0;
	_f = 1;
	_do_map = 0;

	//==============================================================

	/* for sweeperlines copied from vision.cc */

	nav_path__num = 9;
	nav_path__center_path_id = nav_path__num/2;
	nav_path__view_cone__spacing = 16; // degrees
#if USE_PATH_IMG
	danger_per_barrel_pixel = 1; //=1
#else
	danger_per_barrel_pixel = 6; //=1
#endif
	nav_path__path_search_girth = 0; // pixels near curr line to search <- deprecated!
	nav_path__danger_smoothing_radius = nav_path__center_path_id-1; // lines nearby to search
	max_path_danger = 50;//45;
	min_path_danger_value = 99;//20; // lower => be less afraid
	nav_path__view_distance_multiplier = 0.45;//0.5;
	dangerous_pixel_color = CV_RGB(255,0,0);

	worldDebug = cvCreateImage(cvSize(probmap->width,probmap->height),8,3);

	//==============================================================


}

void MapGen::LoadMappingXMLSettings()
{
	/* load xml file */
	XmlConfiguration cfg("Config.xml");

	/* load settings */
	{
		maxFeatures   = cfg.getInt("maxFeatures");
		numFramesBack = cfg.getInt("numFramesBack");
		maxFeatureShift = cfg.getInt("maxFeatureShift");
		moveTo127     = cfg.getInt("moveTo127");

		/* display windows */
		if (cfg.getInt("doMapping"))
		{
#if DO_LANDMARK_MAP
			cvNamedWindow("landmarkmap");
#else
			cvNamedWindow("processmap");
#endif
			cvNamedWindow("curr");
			//cvNamedWindow("probmap");

		}
	}

	/* test */
	{
		if (maxFeatures==-1)
		{
			printf("ERROR: Mapping settings NOT loaded! Using DEFAULTS \n");
			{
				maxFeatures   = 70;
				numFramesBack = 2;
				maxFeatureShift = 25;
				moveTo127     = 0;

			}
		}
		else
		{
			printf("Mapping settings loaded \n");
		}
		printf("\tvalues: maxFeatures %d  \n", maxFeatures);
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
			printf(" %.2f ",(float)cvmGet( matrix, row, col ));
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

void MapGen::mapCamPointToWorldPoint(float camx, float camy, float& worldx, float& worldy)
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
	float thresh = WORLDPT_PIXDIFF ;
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
		float wx,wy;

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
	float x,y,a,b;
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
	cvShowImage("landmarkmap",worldmap);

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
	if (moveTo127)
	{
		float curr;
		float speed = 1;
		int i;

#pragma omp parallel for private(i,curr)
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
	int pad = 4;        // remove noise around img edges
	float badval = -1; // rate for obstacle probability
	float goodval = 2; // rate for travpath probability
	float setval,weight;
	float wx,wy;
	int x,y;

#pragma omp parallel for private(y,x,setval,wx,wy,weight)
	for (y=pad; y<visCvThresh->height-divby-pad; y+=divby)
	{
		for (x=pad; x<visCvThresh->width-divby-pad; x+=divby )
		{
			// check void mask
			if ( cvGet2D(visCvGlutMask,y,x).val[0]==255 )
			{
				// map pts
				mapCamPointToWorldPoint((float)x,(float)y,wx,wy);
				// get current val
				setval = cvGetReal2D(probmap,wy,wx);
				// and add/subtract based on thresh image
#if USE_PATH_IMG
				weight = (cvGetReal2D( /* visCvThresh */ visCvPath  ,y,x )==0)?badval:goodval; // more black
#else
				weight = (cvGetReal2D(  visCvThresh /* visCvPath */ ,y,x )==0)?badval:goodval; // cooler looking
#endif
				// weight closer stuff higher
				//if (y>imgHalfHeight) weight *= 2;
				weight += (weight<0)?(-y/80):(y/80); //80=240/3
				// update value
				setval += weight;
				// cap results
				if (setval>255) setval = 255;
				else if (setval<0) setval = 0;
				// update probmap
				cvSetReal2D(probmap,wy,wx,setval);
			}
		}
	}


	/* get robot orientation in world coordinates */
	float a,b;
	mapCamPointToWorldPoint(imgHalfWidth, 0, a, b); // top center of camera frame
	mapCamPointToWorldPoint(imgHalfWidth, visCvGrey->height - BASE_OFFSET, wx, wy); // bottom center of camera frame (minus some)

	/* update/avg current robot orientation */
	if (1)
	{
		robotBaseAt = cvPoint( (robotBaseAt.x*(1-K_)+wx*K_), (robotBaseAt.y*(1-K_)+wy*K_) ); // turn smoother
		robotLookingAt = cvPoint( (robotLookingAt.x*(1-K_)+a*K_), (robotLookingAt.y*(1-K_)+b*K_) ); // turn smoother
	}
	else
	{
		robotBaseAt = cvPoint( (robotBaseAt.x+wx)/2 , (robotBaseAt.y+wy)/2 ); // turn faster
		robotLookingAt = cvPoint( (robotLookingAt.x+a)/2 , (robotLookingAt.y+b)/2 ); // turn faster
	}

	//printf("bx=%d by=%d  lx=%d ly=%d \n",robotBaseAt.x,robotBaseAt.y,robotLookingAt.x,robotLookingAt.y);

	/* draw a circle denoting base orientaion */
	//cvCircle(probmap, CvPoint(robotBaseAt),
	//		 1,
	//		 CV_RGB(127,127,127),
	//		 1, 8, 0);
	/* display */
	//cvShowImage("probmap",probmap);


	/* checi world map bounds - HACK! */
	int edgepad = 55;
	if (    robotLookingAt.x<edgepad ||
			robotLookingAt.x>probmap->width-edgepad ||
			robotLookingAt.y<edgepad ||
			robotLookingAt.y>probmap->height-edgepad )
	{
		printf("The world is flat! The world is flat! The world is flat! \n");
		init();  // HACK HACK - not fully tested!
		return 0;
	}


	/* success */
	return 1;
}


//  copied from vision.cc visSweeperLines() and modified to work here
int MapGen::processMap(Point2D<int>& goal)
{

	// display
	cvCvtColor(probmap, worldDebug, CV_GRAY2BGR);
	Graphics g_draw(worldDebug);

	// temp vars used below
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
			Point2D<float> pathStart =navPath_start(/*pathID*/);
			Point2D<float> pathEnd = navPath_end(pathID);

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

				//for (int delta=-nav_path__path_search_girth; delta<=nav_path__path_search_girth; delta++)
				{
					// (XXX: Consider *nearby* pixels as different fragments of the path-pixel)
					/*Point2D<int>*/
					curPoint = pathPoints[j];
					//curPoint.x += delta;

					// check path image (half size) for black=bad
					float val = cvGetReal1D(probmap,curPoint.y*probmap->width+curPoint.x);
					if (val <= min_path_danger_value)
					{
						// everything bad is a barrel

						// sum dangers
						//curPixelDanger += danger_per_barrel_pixel;

						// set danger to closeness
						//printf("id %d max %d danger %d\n",pathID,n2,n2-j);
						curPixelDanger = n2-j; //break;
					}
					else
					{
						// Nothing special: Probably grass
					}
				}

				// Compensate for considering *nearby* pixels
				//curPixelDanger /= ((nav_path__path_search_girth*2) + 1);
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
			g_draw.setColor( cvScalar(118,138, (curPathDanger / (float)max_path_danger) * 255) );
			g_draw.drawLine( (int) pathStart.x, (int) pathStart.y, (int) pathEnd.x, (int) pathEnd.y);

			g_draw.setColor(dangerous_pixel_color);
			// Hilight the "dangerous" pixels in the path
			// (that contributed to the path's total danger value)
			for (uint j=0, n2=pathPoints.count(); j<n2; j++)
			{
				uchar curPixelDanger = pixelDanger[j];
				if (curPixelDanger != 0)
				{
					/*Point2D<int>*/
					curPoint = pathPoints[j];

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

		// Copy second edge
//		for (int curPath_id = (nav_path__num - nav_path__danger_smoothing_radius);
//				curPath_id < nav_path__num;
//				curPath_id++)
//		{
//			smoothedPathDangers[curPath_id] = pathDanger[curPath_id];
//		}

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
			g_draw.setColor( cvScalar( (pathDanger[bestPath_id] / (float)max_path_danger) * 255 ,128,128 ) );

			// Redraw the path, but much more thickly (in order to hilight it)
			Point2D<float> bestPath_start = navPath_start(/*bestPath_id*/);
			Point2D<float> bestPath_end = navPath_end(bestPath_id);
//            for (int deltaX = -1; deltaX <= 1; deltaX++)
//            {
//                g_draw.drawLine(
//                    ((int) bestPath_start.x) + deltaX, (int) bestPath_start.y,
//                    ((int) bestPath_end.x) + deltaX, (int) bestPath_end.y);
//            }
			// Loop unrolling for above ^
			g_draw.drawLine(
				((int) bestPath_start.x) + -1, (int) bestPath_start.y,
				((int) bestPath_end.x)   + -1, (int) bestPath_end.y);
			g_draw.drawLine(
				((int) bestPath_start.x)    , (int) bestPath_start.y,
				((int) bestPath_end.x)      , (int) bestPath_end.y);
			g_draw.drawLine(
				((int) bestPath_start.x) + 1, (int) bestPath_start.y,
				((int) bestPath_end.x)   + 1, (int) bestPath_end.y);

			/* Update goal */
			//Point2D<int> goal;
			int scalex = 40;
			goal.x = (nav_path__center_path_id-bestPath_id)*scalex ;
			goal.y = (max_path_danger + 2 - pathDanger[bestPath_id]) * 2 ;

			//printf("goal(%d,%d) \n",goal.x,goal.y); // print in vision.cc
		}

		// DO IN MAIN:
		// 	drive motors
		// 	display motor output

	}

	/* results */
	cvShowImage("processmap",worldDebug);

	return 1;
}

Point2D<float> MapGen::navPath_start(/*int pathID*/)
{
	Point2D<float> start;
	start.x = robotBaseAt.x;
	start.y = robotBaseAt.y;
	return start;
}
Point2D<float> MapGen::navPath_end(int pathID)
{
	return navPath_start(/*pathID*/) + navPath_vector(pathID);
}
Point2D<float> MapGen::navPath_vector(int pathID)
{

	int x = (robotLookingAt.x - robotBaseAt.x) ;
	int y = (robotLookingAt.y - robotBaseAt.y) ; // positive y down
	float rad = (atan((float)y/(float)x)) ; // make up = 0 deg
	float radoff =( (nav_path__center_path_id-pathID)*nav_path__view_cone__spacing )*M_PI/180.0; // path selection
	rad += radoff; // rotate based on path
	if (x<0) rad += -M_PI ;
	//printf("  rad %.2f x %.2f y %.2f \n",rad,x,y);
	float radius = sqrtf(pow(x,2)+pow(y,2)); // dist formula
	return Point2D<float>(
			   radius*cos(rad),
			   radius*sin(rad) )   // opencv y-axis is inverse of math y-axis
		   * nav_path__view_distance_multiplier;
}



