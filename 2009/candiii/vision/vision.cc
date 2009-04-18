#include "vision.h"
#include <stdio.h>
#include "XmlConfiguration.h"
#include "image_buffers.h"
#include "Graphics.h"
#include <omp.h>

/*
 * This file contains the robot's primary vision processing code main function.
 *   by: Chris McClanahan
 *
 */

/* for scanning the path image */
#define BAD_PIXEL  0	// value to set path image to
#define GOOD_PIXEL 255	// value to set path image to
#define L_R_OFFSET 25 	// pixel spacing from center scan line up
#define PIXEL_SKIP 2 	// noise filtering threshold in checkPixel()
#define EDGE_PAD   4    // top/bottom padding

/* for Adapt() */
#define K_ROI 0.2035     // float: percentage of new roi avg to add to running avg


/*
 *
 */
Vision::Vision()
{

}

/*
 *
 */
Vision::~Vision()
{
	/* no memory leaks! */
	cvReleaseImage(&roi_img);
}

/*
 * Performs ALL vision processing. Returns updated goal/heading.
 */
void Vision::visProcessFrame(Point2D<int>& goal)
{

	/* Abort if there is no image. */
//	if ( visCvRaw->imageData==NULL )
//	{
//		printf("visCvRaw is NULL!\n");
//		return;
//	}

	cvSmooth(visCvRawTransform,visCvRawTransform,CV_BLUR,7,0,0,0);

	/* copy image to internal buffer for drawing */
	cvCopy(visCvRawTransform,visCvDebug);

	/* Choose vision algorithm, and update goal.  */
	if (DO_ADAPTIVE)
	{
		visAdaptiveProcessing(goal); // NEW!
	}
	else
	{
		visHsvProcessing(goal); // modified 2008 method
	}


}//end vision processing

/*
 *
 */
void Vision::init()
{

	/* load in vision settings */
	LoadVisionXMLSettings();


	/*** SweeperLines ****************************************************/
	if ( DO_TRANSFORM )
	{
		// Number of paths that are assessed between the starting/ending angles
		nav_path__num = 21; //29;		// (Number of sweeper lines - should be odd number)
		// Proportional to the lengths of the paths (in image space)
		nav_path__view_distance_multiplier = 0.65; 	/* > 0.0 */
	}
	else
	{
		// Number of paths that are assessed between the starting/ending angles
		nav_path__num = 29; //15;		// (Number of sweeper lines - should be odd number)
		// Proportional to the lengths of the paths (in image space)
		nav_path__view_distance_multiplier = .95; 	/* > 0.0 */
	}
	// Defines the "view/navigation cone", which is where the set of
	// considered navigation paths is taken from.
	nav_path__view_cone__offset = 21; //23.0; //30;
	nav_path__view_cone__start_angle = 0.0 + nav_path__view_cone__offset;	// >= 0.0
	nav_path__view_cone__end_angle = 180.0 - nav_path__view_cone__offset;	// <= 180.0
	nav_path__view_cone__delta_angle = nav_path__view_cone__end_angle - nav_path__view_cone__start_angle;
	nav_path__view_cone__spacing = nav_path__view_cone__delta_angle / (nav_path__num-1);
	// XXX: Controls how many pixels *near* a path-pixel are searched for dangerous pixels
	// 0 <= nav_path__path_search_girth
	nav_path__path_search_girth = 1;
	// (do not change without reason!)
	nav_path__center_path_id = (int) round((90.0 - nav_path__view_cone__start_angle) / nav_path__view_cone__spacing);
	// Amount of danger posed by a single barrel-pixel
	// (everything bad is a barrel)
	danger_per_barrel_pixel = 1;
	// Path danger values higher than this will be clipped to this value
	max_path_danger = 79;					// >= 0 // 75
	// smoothing = paths that are *near* dangerous paths are also considered to be dangerous
	nav_path__danger_smoothing_radius = 6;	// >= 0 // 6
	// colors
	dangerous_pixel_color = CV_RGB(255, 0, 0);		// red
	/**********************************************************************************/

	// font
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2);

	// display roi for adaptive coloring
	cvNamedWindow("roi",1);
	// move the window for easy comparison to raw
	cvMoveWindow( "roi", visCvRaw->width/2-20, 60+visCvRaw->height );

	/* init adaptive processing stuff */
	{
		/* define roi corners (upper-left and lower-right) */
		if (DO_TRANSFORM)
		{
			UL = cvPoint(  visCvDebug->width/3+adapt_boxPad, visCvDebug->height-adapt_boxPad/4);
			LR = cvPoint(2*visCvDebug->width/3-adapt_boxPad, visCvDebug->height-adapt_boxPad/7);
		}
		else
		{
			UL = cvPoint(  visCvDebug->width/3+adapt_boxPad, visCvDebug->height-adapt_boxPad/2);
			LR = cvPoint(2*visCvDebug->width/3-adapt_boxPad, visCvDebug->height-adapt_boxPad/5);
		}

		/* setup roi */
		roi.x = UL.x;
		roi.y = UL.y;
		roi.width  = LR.x-UL.x;
		roi.height = LR.y-UL.y;

		/* create and set roi img */
		roi_img = cvCreateImage( cvSize(roi.width, roi.height), IPL_DEPTH_8U, 3 );

		/* starting colors */
		avgB=127;
		avgG=127;
		avgB=127;
	}

}

/*
 * Performs ALL vision processing. Method A
 */
void Vision::visHsvProcessing(Point2D<int>& goal)
{

	/* fix special case colors */
	preProcessColors(visCvDebug);

	/* split images into channels */
	GetHSVChannels();

	/* threshold saturation
	 * (320x240) */
	cvSmooth(visCvSaturation, visCvSaturation);
	Normalize(visCvSaturation);
	ThresholdImage(visCvSaturation, visCvSaturation, satThreshold);
	//cvDilate(visCvSaturation, visCvSaturation, NULL, 1);

	/* threshold hue
	 * (320x240) */
	cvSmooth(visCvHue, visCvHue);
	Normalize(visCvHue);
	ThresholdImage(visCvHue, visCvHue, hueThreshold);
	//cvDilate(visCvHue, visCvHue, NULL, 1);

	/* or the images together
	 * (320x240) */
	cvOr(visCvSaturation, visCvHue, visCvThresh);
	//cvDilate(visCvThresh, visCvThresh, NULL, 1);

	/* make white=good & black=bad
	 * (320x240) */
	cvNot(visCvThresh, visCvThresh);

	/* generate path image, with white=path & black=bad
	 * (320x240) */
	visGenPath(visCvThresh);

	/* find next goal for robot by scanning up visCvPath Image, and set the goal.
	 * (320x240) */
	robotWidthScan(visCvPath, goal_far);	// returns -1 on error

	/* setup navigation lines that sweep across the screen and updates goal. */
	visSweeperLines(goal_near);

	/* update return goal */
	CvtPixToGoal(goal);

}

/*
 * This function sets up navigation path lines
 * (sweeping out from the bottom center of the image)
 * and scans those lines
 * (in the visCvPath image)
 * to see if we're on path or not.
 *
 * The goal point passed in is set/updated in this function
 */
void Vision::visSweeperLines(Point2D<int>& goal)
{

	Graphics g_draw(visCvDebug);
	int pathDanger[nav_path__num];
	int curPixelDanger = 0;
	int curPathDanger = 0;
	int weight = 0;
	int pathID = 0;

	/* Compute and draw all navigation paths we are considering (and do other actions) */
	{
#pragma omp parallel for private(curPathDanger,curPixelDanger,weight,pathID)
		for (pathID=0; pathID<nav_path__num; pathID++)
		{
			// Calculate path parameters
			Point2D<double> pathStart = navPath_start(pathID);
			Point2D<double> pathEnd = navPath_end(pathID);

			// Calculate the set of points in the path
			QVector< Point2D<int> > pathPoints;
			Graphics::calculatePointsInLine(
				(int) pathStart.x, (int) pathStart.y,
				(int) pathEnd.x, (int) pathEnd.y,			// @ 640x480: 	maxPathEnd.x=607
				&pathPoints);								//				minPathEnd.x=112

			// Calculate the danger value for the path along with
			// the danger contributions of all the pixels in the path
			curPathDanger = 0;
			uchar pixelDanger[pathPoints.count()];
			Point2D<int> curPoint;
			for (uint j=0, n2=pathPoints.count(); j<n2; j++)
			{
				// Calculate the danger contribution of the current path-pixel to the path-danger
				curPixelDanger = 0;
				for (int delta=-nav_path__path_search_girth; delta<=nav_path__path_search_girth; delta++)
				{
					// (XXX: Consider *nearby* pixels as different fragments of the path-pixel)
					/*Point2D<int>*/
					curPoint = pathPoints[j];
					curPoint.x += delta;

					// check path image (half size) for black=bad
					if (visCvPath->imageData[curPoint.y/2*visCvPath->width+curPoint.x/2]==0)
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
			weight = abs(nav_path__center_path_id-pathID)/2;
			curPathDanger += weight;

			// Clip high danger values to be no higher than max_path_danger
			if (curPathDanger > max_path_danger)
			{
				curPathDanger = max_path_danger;
			}
			pathDanger[pathID] = curPathDanger;

			// Draw the body of the path using a color that is determined from the path's danger value
			g_draw.setColor(navPath_color(curPathDanger));
			g_draw.drawLine( (int) pathStart.x, (int) pathStart.y, (int) pathEnd.x, (int) pathEnd.y);

			// Hilight the "dangerous" pixels in the path
			// (that contributed to the path's total danger value)
			g_draw.setColor(dangerous_pixel_color);
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
						g_draw.drawPixel(curPoint.x, curPoint.y);
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
			int bestPath_distanceFromCenter = abs(nav_path__center_path_id - bestPath_id);
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
			g_draw.setColor(navPath_color(pathDanger[bestPath_id]));

			// Redraw the path, but much more thickly (in order to hilight it)
			Point2D<double> bestPath_start = navPath_start(bestPath_id);
			Point2D<double> bestPath_end = navPath_end(bestPath_id);
			for (int deltaX = -1; deltaX <= 1; deltaX++)
			{
				g_draw.drawLine(
					((int) bestPath_start.x) + deltaX, (int) bestPath_start.y,
					((int) bestPath_end.x)   + deltaX, (int) bestPath_end.y);
			}

			/* Update goal
			 * (convert to 320x240 frame) */
			goal.x = (bestPath_id) * visCvPath->width / (nav_path__num-1); //(int)bestPath_end.x/2;
			goal.y = max_path_danger - pathDanger[bestPath_id]; //(int)bestPath_end.y/2;

			//printf("goal(%d,%d) \n",goal.x,goal.y); // print in vision.cc
		}

		// DO IN MAIN:
		// 	drive motors
		// 	display motor output

	}

}

/*
 * This function scans from the bottom center of image upward,
 *	checking to see if the width of the robot can progress
 *	any further up the image, while sliding left/right as needed
 */
void Vision::robotWidthScan(IplImage* img, Point2D<int>& goal)
{

	int center = img->width/2;
	int startx = center - ROBOT_WIDTH/2;
	int endx = startx + ROBOT_WIDTH;
	int y = img->height-EDGE_PAD;
	int x = startx;
	int half = ROBOT_WIDTH/2;
	int i;

	// return -1 on failure
	goal.x=-1;
	goal.y=-1;

	/* 	scan from bottom center of image upward,
		checking to see if the width of the robot can progress
		any further up the image, sliding left/right as needed
	*/
	for ( ; y >= EDGE_PAD ; y-- )
	{

		//check left, move right
		for (x=startx; x<img->width-ROBOT_WIDTH-1; x++)
		{
			if (!checkPixel(img,x,y)|| !checkPixel(img,x+half,y) )
			{
				x++;	//slide right
				startx=x;
				endx=x+ROBOT_WIDTH;
			}
			else
			{
				startx=x;
				endx=x+ROBOT_WIDTH;
				break;
			}
		}

		for ( i = startx; i<endx; i++)   //scan along width to not cross over boundary
		{
			if (!checkPixel(img,i,y))
			{
				break;
			}
			else
			{
				goal.x = i-half;
				goal.y = y;
			}
		}
		if (i>=endx)
		{
			//success, keep going up
			continue;
		}

		//check right, move left
		for (x=endx; x>0+ROBOT_WIDTH; x--)
		{
			if (!checkPixel(img,x,y) || !checkPixel(img, x-half, y) )
			{
				x--;	//slide left
				startx=x-ROBOT_WIDTH;
				endx=x;
			}
			else
			{
				startx=x-ROBOT_WIDTH;
				endx=x;
				break;
			}
		}
		for ( i = endx; i>startx; i--)   //scan along width to not cross over boundary
		{
			if (!checkPixel(img,i,y))
			{
				break;
			}
			else
			{
				goal.x = i+half;
				goal.y = y;
			}
		}
		if (i<=startx)
		{
			//success, keep going up
			continue;
		}
		else
		{
			break;		//we dont fit left or right
		}
	}

	//sanity check goal
	if (goal.x==-1||goal.y==-1)
	{
		//not good
	}
	else
	{
		//found goal
		if (!checkPixel(img,goal.x,goal.y))
		{
			goal.x=goal.y=-1;	//not good, error in scanning
		}
		else
		{
			/* GOAL! */
			//=== visCvPath is 320x240 so convert to 640x480 ===//
			int gy2 = goal.y*2;
			int gx2 = goal.x*2;
			Graphics g(visCvDebug);
			g.setColor(CV_RGB(20,20,0));
			g.drawLine( img->width, img->height*2-2,
						gx2, gy2 );
			g.drawLine( gx2-ROBOT_WIDTH ,gy2,
						gx2+ROBOT_WIDTH, gy2 );
			//==================================================//
		}
	}

	// IMPLICITLY:
	// 	return goalx,goaly,
	// 	or set -1 on error

}

/*
 * This function generates the visCvPath image by scanning the
 *   visCvThresh image from the bottom center up, while scanning/filling left and right.
 * As soon as the center pixel is bad, all the above pixels are bad too.
 */
void Vision::visGenPath(IplImage* img)
{
	int width = img->width;
	int height = img->height-EDGE_PAD; // skip noise at bottom
	int goodFirst = 1;
	int x = width/2; // default
	int blackout = 0;

	// find best column to scan up in
	x = findBestX(img, height, x);

	// scan bottom to top in rows; white = path; black = bad
	// skip the bottom few pixels, due to noise
	for (int y = height; y >= EDGE_PAD ; y--)
	{
		if (checkPixel(img,x,y))  	//check starting point in middle
		{
			goodFirst=1;
		}
		else
		{
			goodFirst=0;
			blackout=1;
		}

		//scan left then right & generate visCvPath image
#pragma omp parallel
		{
			scanFillLeft  (visCvPath, x, y, goodFirst, 0      , blackout);
			scanFillRight (visCvPath, x, y, goodFirst, width-1, blackout);
		}

	}//y

	// debug draw black line to see which column was chosen (left,center,right)
	//cvLine(visCvThresh, cvPoint(x,1), cvPoint(x,height-2), (CV_RGB(0,0,0)), 2, 8, 0);

}//visGenPath

/*
 * This function scans up the center (and left/right of center) of thresh image,
 *   and returns the column that went the 'hightest' in the image
 */
int Vision::findBestX(IplImage* img, int height, int center)
{
	int left  = center - L_R_OFFSET;
	int right = center + L_R_OFFSET;
	int heightL = 0;
	int heightR = 0;
	int heightC = 0;
	int y;

	// break as soon as we see an obstacle
	for (y = height; y >= EDGE_PAD ; y--)
	{
		if (checkPixel(img,left,y))
			heightL++;
		else
			break;
	}
	for (y = height; y >= EDGE_PAD ; y--)
	{
		if (checkPixel(img,center,y))
			heightC++;
		else
			break;
	}
	for (y = height; y >= EDGE_PAD ; y--)
	{
		if (checkPixel(img,right,y))
			heightR++;
		else
			break;
	}

	if ( (heightL > heightC) && (heightL > heightR) )
		return left;
	else if ( (heightR > heightC) && (heightR > heightL) )
		return right;
	else
		return center;    // default to center

}

/*
 * This function retuns T/F based on:
 * 		white = good/path
 * 		black = bad/obstacle
 */
int Vision::checkPixel(IplImage* img, const int x, const int y)
{
	// check: black = bad
	//if ( !val )
	if (!(unsigned char)img->imageData[y*img->width+x] )
	{
		// check for noise
		if ( !(img->imageData[ (y-PIXEL_SKIP)*img->width+x ]) )
		{
			//good = 0;
			return 0;
		}
		else
		{
			//good = 1; // just a small spot, keep going up
			return 1;
		}
	}
	// white = good
	else
	{
		//good = 1;
		return 1;
	}
}

/*
 *  This function scans from the center of the visCvThresh image to end
 *  	checking for good / bad pixels, setting the visCvPath image accordingly
 */
void Vision::scanFillLeft(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout)
{
	int x=middleX;
	int good;
	int index;
	int offset = y*img->width;

	if (blackout)
	{
		for (; x>=end; x--)  	//fill black
		{
			//set bad
			index = offset+x;
			img->imageData[index] = BAD_PIXEL;
		}
		return;
	}

	if (goodFirst)  		//starting pixel is good
	{
		good=1;
		for (; x>=end; x--)  	//scan left and check
		{
			if (good)
			{
				if (checkPixel(visCvThresh,x,y))
				{
					//set good
					index = offset+x;
					img->imageData[index] = GOOD_PIXEL;
				}
				else
				{
					//set bad
					good=0;
					index = offset+x;
					img->imageData[index] = BAD_PIXEL;
				}
			}
			else  		//all the rest are bad
			{
				//set bad
				index = offset+x;
				img->imageData[index] = BAD_PIXEL;
			}
		}
	}
	else  		//starting pixel is bad
	{
		good=2;
		for (; x>=end; x--)  	//scan left and check
		{
			if (good==2)  	//in bad spot, check for good spot
			{
				if (checkPixel(visCvThresh,x,y))
				{
					//set good
					good=1;
					index = offset+x;
					img->imageData[index] = GOOD_PIXEL;
				}
				else
				{
					//set bad
					index = offset+x;
					img->imageData[index] = BAD_PIXEL;
				}
			}
			else if (good==1)  	//a good spot appeared, check for bad again
			{
				if (checkPixel(visCvThresh,x,y))
				{
					//set good
					index = offset+x;
					img->imageData[index] = GOOD_PIXEL;
				}
				else
				{
					//set bad
					good=0;
					index = offset+x;
					img->imageData[index] = BAD_PIXEL;
				}
			}
			else   //all the rest are bad
			{
				//set bad
				index = offset+x;
				img->imageData[index] = BAD_PIXEL;
			}
		}
	}
}

/*
 *  This function scans from the center of the visCvThresh image to end
 *  	checking for good / bad pixels, setting the visCvPath image accordingly
 */
void Vision::scanFillRight(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout)
{
	int x=middleX;
	int good;
	int index;
	int offset = y*img->width;

	if (blackout)
	{
		for (; x<end; x++)  	//fill black
		{
			//set bad
			index = offset+x;
			img->imageData[index] = BAD_PIXEL;
		}
		return;
	}

	if (goodFirst)  		//starting pixel is good
	{
		good=1;
		for (; x<end; x++)  	//scan right and check
		{
			if (good)
			{
				if (checkPixel(visCvThresh,x,y))
				{
					//set good
					index = offset+x;
					img->imageData[index] = GOOD_PIXEL;
				}
				else
				{
					//set bad
					good=0;
					index = offset+x;
					img->imageData[index] = BAD_PIXEL;
				}
			}
			else  		//all the rest are bad
			{
				//set bad
				index = offset+x;
				img->imageData[index] = BAD_PIXEL;
			}
		}
	}
	else  		//starting pixel is bad
	{
		good=2;
		for (; x<end; x++)  	//scan right and check
		{
			if (good==2)  	//in bad spot, check for good spot
			{
				if (checkPixel(visCvThresh,x,y))
				{
					//set good
					good=1;
					index = offset+x;
					img->imageData[index] = GOOD_PIXEL;
				}
				else
				{
					//set bad
					index = offset+x;
					img->imageData[index] = BAD_PIXEL;
				}
			}
			else if (good==1)  	//a good spot appeared, check for bad again
			{
				if (checkPixel(visCvThresh,x,y))
				{
					//set good
					index = offset+x;
					img->imageData[index] = GOOD_PIXEL;
				}
				else
				{
					//set bad
					good=0;
					index = offset+x;
					img->imageData[index] = BAD_PIXEL;
				}
			}
			else   //all the rest are bad
			{
				//set bad
				index = offset+x;
				img->imageData[index] = BAD_PIXEL;
			}
		}
	}
}

/*
 * Manually check each pixel for special cases not accounted for via HSV
 *
 */
void Vision::preProcessColors(IplImage* img)
{
	unsigned char red,green,blue;

	//TODO: optimize with openmp and pointers!

	// pixel data loop
	for (int i = 0; i < img->imageSize-3; i+=3)
	{

		// get pixel data
		blue  = img->imageData[i  ];
		green = img->imageData[i+1];
		red   = img->imageData[i+2];

		// check for neon green
		if ( (green>205) && (green>red) && (green>blue) && (red>blue) )  //&& (blue<65) ){
		{
			// make orange
			img->imageData[i  ] = 0;
			img->imageData[i+1] = 165;
			img->imageData[i+2] = 255;
		}

	}// end loop

}

/*
 * Converts a measurement in degrees to radians.
 */
double Vision::deg2rad(double degrees)
{
	return degrees * M_PI / 180.0;
}

/*
 *
 */
Point2D<double> Vision::navPath_start(int pathID)
{
	return Point2D<double>(
			   (visCvRaw->width / 2) + (nav_path__center_path_id-pathID),
			   visCvRaw->height - EDGE_PAD -1 );
}

/*
 *
 */
Point2D<double> Vision::navPath_vector(int pathID)
{
	//double deg = navPath_angle(pathID);
	//double rad = deg2rad(deg);
	double rad = navPath_angle(pathID)*M_PI/180.0;
	double radius = 0.75 * (visCvRaw->width) / 2;
	return Point2D<double>(
			   radius*cos(rad),
			   -radius*sin(rad))	// computer y-axis is inverse of math y-axis
		   * nav_path__view_distance_multiplier;
}

/*
 *
 */
Point2D<double> Vision::navPath_end(int pathID)
{
	return navPath_start(pathID) + navPath_vector(pathID);
}

/*
 *  result is in degrees
 */
double Vision::navPath_angle(int pathID)
{
	return  nav_path__view_cone__start_angle +
			(pathID * nav_path__view_cone__spacing);
}

/*
 *
 */
CvScalar Vision::navPath_color(int pathDanger)
{
	// levels
	return cvScalar(
			   178 ,
			   32 ,
			   (pathDanger / (double)max_path_danger) * 255
		   );
}

/*
 * image display control
 */
void Vision::ConvertAllImageViews(int trackbarVal)
{

	switch (trackbarVal)
	{
	case 0:
		cvPutText(visCvRaw, "Raw", cvPoint(5,visCvRaw->height-10), &font, CV_RGB(255,255,255));
		cvShowImage("display", visCvRaw);
		break;
	case 1:
		cvPutText(visCvDebug, "Debug", cvPoint(5,visCvRaw->height-10), &font, CV_RGB(255,255,255));
		cvShowImage("display", visCvDebug);
		break;
	case 2:
		if (!DO_TRANSFORM) cvPutText(visCvPath, "Path", cvPoint(5,visCvPath->height-10), &font, CV_RGB(0,0,0));
		else cvPutText(visCvPath, "Path", cvPoint(5,visCvPath->height-10), &font, CV_RGB(255,255,255));
		cvShowImage("display", visCvPath);
		break;
	case 3:
		if (!DO_TRANSFORM) cvPutText(visCvThresh, "Thresh", cvPoint(5,visCvThresh->height-10), &font, CV_RGB(0,0,0));
		else cvPutText(visCvThresh, "Thresh", cvPoint(5,visCvThresh->height-10), &font, CV_RGB(255,255,255));
		cvShowImage("display", visCvThresh);
		break;
	case 4:
		if (!DO_TRANSFORM) cvPutText(visCvAdapt, "Adaptive", cvPoint(5,visCvAdapt->height-10), &font, CV_RGB(0,0,0));
		else cvPutText(visCvAdapt, "Adaptive", cvPoint(5,visCvRaw->height-10), &font, CV_RGB(255,255,255));
		cvShowImage("display", visCvAdapt);
		break;
	case 5:
		cvPutText(visCvGrey, "Grey", cvPoint(5,visCvGrey->height-10), &font, CV_RGB(255,255,255));
		cvShowImage("display", visCvGrey);
		break;
	case 6:
		if (DO_TRANSFORM) cvPutText(visCvHue, "Hue", cvPoint(5,visCvHue->height-10), &font, CV_RGB(0,0,0));
		else cvPutText(visCvHue, "Hue", cvPoint(5,visCvHue->height-10), &font, CV_RGB(255,255,255));
		cvShowImage("display", visCvHue);
		break;
	case 7:
		if (DO_TRANSFORM) cvPutText(visCvSaturation, "Sat", cvPoint(5,visCvSaturation->height-10), &font, CV_RGB(0,0,0));
		else cvPutText(visCvSaturation, "Sat", cvPoint(5,visCvSaturation->height-10), &font, CV_RGB(255,255,255));
		cvShowImage("display", visCvSaturation);
		break;
	case 8:
		cvPutText(visCvHSV, "HSV", cvPoint(5,visCvRaw->height-10), &font, CV_RGB(255,255,255));
		cvShowImage("display", visCvHSV);
		break;

		/* future use (change value of numberOfViews) */
//  case 9:
//  	cvShowImage("display", );
//  	break;

	default:
		break;

	}
	//cvWaitKey(10);
	cvWaitKey(10);

}

/*
 *
 */
void Vision::LoadVisionXMLSettings()
{
	/* load xml file */
	XmlConfiguration cfg("Config.xml");

	/* load settings */
	{
		satThreshold = cfg.getInt("satThresh");
		hueThreshold = cfg.getInt("hueThresh");
		DO_TRANSFORM = cfg.getInt("doTransform");
		ROBOT_WIDTH  = cfg.getInt("robotWidth");
		adapt_maxDiff= cfg.getInt("maxDiff");
		adapt_boxPad = cfg.getInt("boxPadding");
		DO_ADAPTIVE  = cfg.getInt("doAdaptive");
	}

	/* test */
	{
		if (satThreshold==-1 || hueThreshold==-1)
		{
			printf("ERROR: Vision settings NOT loaded! Using DEFAULTS \n");
			{
				satThreshold = 60;
				hueThreshold = 20;
				DO_TRANSFORM = 1;
				ROBOT_WIDTH  = 30;
				adapt_maxDiff= 35;
				adapt_boxPad = 100;
				DO_ADAPTIVE  = 1;
			}
		}
		else
		{
			printf("Vision settings loaded \n");
		}
		printf("thresholds: sat %d  hue %d \n",satThreshold,hueThreshold);
		printf("thresholds: maxDiff %d  boxPad %d \n",adapt_maxDiff,adapt_boxPad);
	}


	/* temporary hack to use longer sweeperlines */
	//DO_TRANSFORM = 0;


}

/*
 *
 */
void Vision::GetRGBChannels()
{
	// splitt raw image into color channels
	cvSplit(visCvRawTransform, visCvBlueChannel, visCvGreenChannel, visCvRedChannel, NULL);
}

/*
 *
 */
void Vision::GetHSVChannels()
{
	// calculate HSV - Hue Saturation Value(Greyscale)
	cvCvtColor(visCvRawTransform, visCvHSV, CV_BGR2HSV);
	// shrink !!
	cvResize(visCvHSV, visCvHSVSmall, CV_INTER_LINEAR);
	// hsv
	cvCvtPixToPlane(visCvHSVSmall, visCvHue, visCvSaturation, visCvGrey, 0);
}

/*
 *
 */
void Vision::ThresholdImage(IplImage *src, IplImage *dst, int thresh)
{
	// binary threshold
	cvThreshold(src,dst,thresh,255,CV_THRESH_BINARY_INV);
}

/*
 * Normalizes a 0-255 grayscale image using its min/max values
 */
void Vision::Normalize(IplImage* img)
{
	double min,max,scale;
	cvMinMaxLoc(img, &min, &max, NULL, NULL, NULL);
	if (max != min)
	{
		scale = 255.0/(max-min);
	}
	else
	{
		scale = 1.0;
	}
	cvScale( img, img, scale, -scale*min ); //Normalizes matrix to 0-255 (grayscale)
}

/*
 * Normalizes a 0-255 grayscale image using its histogram
 *  (normalizes brightness and increases contrast of the image)
 */
void Vision::Equalize(IplImage* img)
{
	if (img->nChannels == 1)
	{
		cvEqualizeHist(img,img);
	}
}

/*
 * Convert img coordinates to heading/goal coordinates
 */
void Vision::CvtPixToGoal(Point2D<int>& goal)
{

	int closeness = visCvPath->height/3; // smaller value => use sweeper lines more often
	float farweight; // weight of the far goal

	/* weight goals depending on how far we can see */
	if ( goal_far.y > closeness ) // y is inverted -> 0 is top&far
	{
		/* can't see very far */
		farweight = 0.00;
	}
	else
	{
		/* can see far off */
		farweight = 0.10;
	}

	/* set averaged goals */
	goal.y = farweight*goal_far.y + (1-farweight)*goal_near.y;
	// (flip goal_far.x's sign)
	goal.x = farweight*(visCvPath->width-goal_far.x) + (1-farweight)*goal_near.x;



	/* convert goal to heading:
	 * x = rotational speed ; range = (-128,127)
	 * y = forward speed    ; range = (0,255)
	 */
	{
		// rotation (0 = go straight)
		goal.x = (visCvPath->width/2 - goal.x) * (255*2) / (visCvPath->width );
		// fwd speed
		goal.y = (visCvPath->height  - goal.y) * (255) / (visCvPath->height);

		/* Now we are using above motor ranges. */
		/* Check for errors and prevent the robot from going crazy */
		if ( (goal.y>=250 && goal.x<=-127) ) // necessary check
		{
			goal.y=10; // min fwd speed
			goal.x=0;
		}
		else if ( (visCvPath->height-goal_far.y) < 15 ) // XXX: HACK to get onto ramps & yellow bars
		{
			goal.y=10; // min fwd speed
			goal.x=0;
		}

        //goal.x = (float)goal.x * 2;
		// Debug print
		//printf("heading: rot(x): %d 	fwd(y): %d \n",goal.x,goal.y);
	}
}


/*
 * Setup a ROI image region to calculate the average red, green, blue values in that area.
 * Use these averages to compare every pixel to check if it is 'close' to that color.
 * Then generate the visCvAdapt image that will turn into visCvPath image.
 */
void Vision::Adapt()
{

	/* set the roi img */
	cvSetImageROI(visCvRawTransform,roi);
	cvCopy(visCvRawTransform,roi_img);
	cvResetImageROI(visCvRawTransform);

	/* blur image data */
	cvSmooth(roi_img,roi_img,CV_BLUR,7,0,0,0);

	/* display roi box in separate window */
	cvShowImage( "roi" , roi_img );

	/* get average rgb values inside the roi */
	int blue=0,green=0,red=0;
	unsigned char ab,ag,ar;
	CvScalar data = cvAvg(roi_img);
	blue  = data.val[0];
	green = data.val[1];
	red   = data.val[2];

	/* average rgb over time */
	static int first = 1;
	//float k = 0.025; // % of new value to use
	if (first)
	{
		first = 0;
		avgB = blue;
		avgG = green;
		avgR = red;
	}
	else
	{
		avgB = blue*(K_ROI)  + avgB*(1-K_ROI);
		avgG = green*(K_ROI) + avgG*(1-K_ROI);
		avgR = red*(K_ROI)   + avgR*(1-K_ROI);
	}

	unsigned char* rgbdata = (unsigned char*) visCvRawTransform->imageData;
	/* generate visCvAdapt image here!
	 *  white=good ~ black=bad */
	for (int i=0; i<visCvRawTransform->imageSize-3; i+=3)
	{
		ab = *(rgbdata  );
		ag = *(rgbdata+1);
		ar = *(rgbdata+2);
		rgbdata+=3;

		if (    (abs(ab-(unsigned char)avgB)<adapt_maxDiff) &&
				(abs(ag-(unsigned char)avgG)<adapt_maxDiff) &&
				(abs(ar-(unsigned char)avgR)<adapt_maxDiff)
		   )
		{
			visCvAdapt->imageData[i/3] = GOOD_PIXEL;
		}
		else
		{
			visCvAdapt->imageData[i/3] = BAD_PIXEL;
		}
	}

	/* display roi box in the image we're using */
	cvRectangle( visCvDebug, UL, LR, CV_RGB(100,0,0), 2, 8, 0);
}

/*
 * Performs ALL vision processing. Method B
 */
void Vision::visAdaptiveProcessing(Point2D<int>& goal)
{

	/* generate visCvAdapt img */
	Adapt();
	cvDilate(visCvAdapt, visCvAdapt, NULL, 1); // removes black spots/noise

	/* shrink visCvAdapt img to 320x240 */
	cvResize(visCvAdapt, visCvThresh, CV_INTER_LINEAR);
	//cvErode(visCvThresh, visCvThresh, NULL, 1); // fills in barrels/lines, but adds grass noise
	//cvDilate(visCvThresh, visCvThresh, NULL, 1); // removes black spots/noise

	/* generate visCvPath */
	visGenPath(visCvThresh);

	/* scan high in img */
	robotWidthScan(visCvPath, goal_far);

	/* setup navigation lines that sweep across the screen and updates goal. */
	visSweeperLines(goal_near);

	/* update return goal */
	CvtPixToGoal(goal);

}


