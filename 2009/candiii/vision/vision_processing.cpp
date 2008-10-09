#include "vision_processing.h"
#include "image_buffers.h"
#include "pathplan/PathPlan.h"
#include <stdio.h>
#include "Graphics.h"
#include "Robot.h" // for DO_TRANSFORM define


/*
 * This file should contain most of the vision processing algorithms,
 *   to be used in vision.cc
 *
 * Only IplImages should be used in this file!
 */


/*** SweeperLines ****************************************************/
#if DO_TRANSFORM
// Number of paths that are assessed between the starting/ending angles
const int NAV_PATH__NUM = 15; //29;		// (Number of sweeper lines)
// Proportional to the lengths of the paths (in image space)	//0.35;<-with-transform
const double NAV_PATH__VIEW_DISTANCE_MULTIPLIER = 0.35; 		//1.00;<-without-transform	/* > 0.0 */
#else
// Number of paths that are assessed between the starting/ending angles
const int NAV_PATH__NUM = 29; //15;		// (Number of sweeper lines)
// Proportional to the lengths of the paths (in image space)	//0.35;<-with-transform
const double NAV_PATH__VIEW_DISTANCE_MULTIPLIER = 1.00; 		//1.00;<-without-transform	/* > 0.0 */
#endif

// Defines the "view/navigation cone", which is where the set of
// considered navigation paths is taken from.
const double NAV_PATH__VIEW_CONE__OFFSET = 30; //23.0; //30;<-without-transform
const double NAV_PATH__VIEW_CONE__START_ANGLE = 0.0 + NAV_PATH__VIEW_CONE__OFFSET;	// >= 0.0
const double NAV_PATH__VIEW_CONE__END_ANGLE = 180.0 - NAV_PATH__VIEW_CONE__OFFSET;	// <= 180.0
const double NAV_PATH__VIEW_CONE__DELTA_ANGLE = NAV_PATH__VIEW_CONE__END_ANGLE - NAV_PATH__VIEW_CONE__START_ANGLE;
const double NAV_PATH__VIEW_CONE__SPACING = NAV_PATH__VIEW_CONE__DELTA_ANGLE / (NAV_PATH__NUM-1);

// XXX: Controls how many pixels *near* a path-pixel are searched for dangerous pixels
// 0 <= NAV_PATH__PATH_SEARCH_GIRTH
const int NAV_PATH__PATH_SEARCH_GIRTH = 1;
// (do not change without reason!)
const int NAV_PATH__CENTER_PATH_ID = (int) round((90.0 - NAV_PATH__VIEW_CONE__START_ANGLE) / NAV_PATH__VIEW_CONE__SPACING);

// Amount of danger posed by a single barrel-pixel
// (everything bad is a barrel)
const int DANGER_PER_BARREL_PIXEL = 1;
// Path danger values higher than this will be clipped to this value
const int MAX_PATH_DANGER = 50;					// >= 0
const int NAV_PATH__DANGER_SMOOTHING__RADIUS = 5;		// >= 0
// colors
const CvScalar MIN_PATH_DANGER_COLOR = CV_RGB(255, 255, 0);		// yellow
const CvScalar MAX_PATH_DANGER_COLOR = CV_RGB(0, 0, 0);		// black
const CvScalar DANGEROUS_PIXEL_COLOR = CV_RGB(0, 0, 255);		// blue

/**********************************************************************************/

// for scanning path image ////////////////////
#define BAD_PIXEL 0				// value to set path image to
#define GOOD_PIXEL 255		// value to set path image to
#define L_R_OFFSET 30			// pixel spacing from center scan line up
#define ROBOT_WIDTH 34 	// pixels wide
#define PIXEL_SKIP 4 	// noise filtering threshold in checkPixel() & also top/bottom padding

// "path planner"
PathPlan planner; // the navigation class object


// FUNCTIONS //////////////////////////////////////////////////////////////////////

void GetRGBChannels() {
    // splitt raw image into color channels
    cvSplit(visCvRaw, visCvBlueChannel, visCvGreenChannel, visCvRedChannel, NULL);
}

void GetHSVChannels() {
    // calculate HSV - Hue Saturation Value(Greyscale)
    cvCvtColor(visCvRaw, visCvHSV, CV_BGR2HSV);
    // shrink !!
    cvResize(visCvHSV, visCvHSVSmall, CV_INTER_LINEAR);
    // hsv
    cvCvtPixToPlane(visCvHSVSmall, visCvHue, visCvSaturation, visCvGrey, 0);
}

void ThresholdImage(IplImage *src, IplImage *dst, int thresh) {
    // binary threshold
    cvThreshold(src,dst,thresh,255,CV_THRESH_BINARY_INV);
}


/*
 * This function generates the visCvPath image by scanning the
 *   visCvThresh image from the bottom center up, while scanning/filling left and right.
 * As soon as the center pixel is bad, all the above pixels are bad too.
 */
void visGenPath(IplImage* img) {
    int width = img->width;
    int height = img->height-PIXEL_SKIP; // skip noise at bottom
    int goodFirst = 1;
    int x = width/2;
    int blackout = 0;

    // find best column to scan up in
    x = findBestX(img, height, x);

    // scan bottom to top in rows; white = path; black = bad
    // skip the bottom few pixels, due to noise
    for (int y = height; y >= PIXEL_SKIP ; y--) {
        if (checkPixel(img,x,y)) {	//check starting point in middle
            goodFirst=1;
        } else {
            goodFirst=0;
            blackout=1;
        }

        //scan left then right & generate visCvPath image
        scanFillLeft  (visCvPath, x, y, goodFirst, 0      , blackout);
        scanFillRight (visCvPath, x, y, goodFirst, width-1, blackout);

    }//y

    // debug draw black line to see which column was chosen (left,center,right)
    //cvLine(visCvThresh, cvPoint(x,1), cvPoint(x,height-2), (CV_RGB(0,0,0)), 2, 8, 0);

}//visGenPath

/*
 * This function scans up the center (and left/right of center) of thresh image,
 *   and returns the column that went the 'hightest' in the image
 */
int findBestX(IplImage* img, int height, int center) {
    int left  = center - L_R_OFFSET;
    int right = center + L_R_OFFSET;
    int heightL = 0;
    int heightR = 0;
    int heightC = 0;
    int y = height;

    // break as soon as we see an obstacle
    for (y = height; y >= PIXEL_SKIP ; y--) {
        if (checkPixel(img,left,y))
            heightL++;
        else
            break;
    }
    for (y = height; y >= PIXEL_SKIP ; y--) {
        if (checkPixel(img,center,y))
            heightC++;
        else
            break;
    }
    for (y = height; y >= PIXEL_SKIP ; y--) {
        if (checkPixel(img,right,y))
            heightR++;
        else
            break;
    }

    if ( heightL > heightC && heightL > heightR )
        return left;
    else if ( heightR > heightC && heightR > heightL )
        return right;
    else
        return center;    // default to center

}

/*
 * This function retuns T/F based on:
 * 		white = good/path
 * 		black = bad/obstacle
 */
int checkPixel(IplImage* img, int x, int y) {
    int good = 0;
    //int index = y*img->width+x;

    unsigned char val = img->imageData[y*img->width+x];

    // check: black = bad
    if ( !val ) {
        // check for noise
        if ( !(img->imageData[ (y-PIXEL_SKIP)*img->width+x ]) )
            good = 0;
        else
            good = 1; // just a small spot, keep going up
    }
    // white = good
    else {
        good = 1;

    }

    return good;
}

/*
 *  This function scans from the center of the visCvThresh image to end
 *  	checking for good / bad pixels, setting the visCvPath image accordingly
 */
void scanFillLeft(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout) {
    int x=middleX;
    int good;
    int index;
    int offset = y*img->width;

    if (blackout) {
        for (;x>=end;x--) {	//fill black
            //set bad
            index = offset+x;
            img->imageData[index] = BAD_PIXEL;
        }
        return;
    }

    if (goodFirst) {		//starting pixel is good
        good=1;
        for (;x>=end;x--) {	//scan left and check
            if (good) {
                if (checkPixel(visCvThresh,x,y)) {
                    //set good
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                } else {
                    //set bad
                    good=0;
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            } else {		//all the rest are bad
                //set bad
                index = offset+x;
                img->imageData[index] = BAD_PIXEL;
            }
        }
    } else {		//starting pixel is bad
        good=2;
        for (;x>=end;x--) {	//scan left and check
            if (good==2) {	//in bad spot, check for good spot
                if (checkPixel(visCvThresh,x,y)) {
                    //set good
                    good=1;
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                } else {
                    //set bad
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            } else if (good==1) {	//a good spot appeared, check for bad again
                if (checkPixel(visCvThresh,x,y)) {
                    //set good
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                } else {
                    //set bad
                    good=0;
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            } else { //all the rest are bad
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
void scanFillRight(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout) {
    int x=middleX;
    int good;
    int index;
    int offset = y*img->width;

    if (blackout) {
        for (;x<end;x++) {	//fill black
            //set bad
            index = offset+x;
            img->imageData[index] = BAD_PIXEL;
        }
        return;
    }

    if (goodFirst) {		//starting pixel is good
        good=1;
        for (;x<end;x++) {	//scan right and check
            if (good) {
                if (checkPixel(visCvThresh,x,y)) {
                    //set good
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                } else {
                    //set bad
                    good=0;
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            } else {		//all the rest are bad
                //set bad
                index = offset+x;
                img->imageData[index] = BAD_PIXEL;
            }
        }
    } else {		//starting pixel is bad
        good=2;
        for (;x<end;x++) {	//scan right and check
            if (good==2) {	//in bad spot, check for good spot
                if (checkPixel(visCvThresh,x,y)) {
                    //set good
                    good=1;
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                } else {
                    //set bad
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            } else if (good==1) {	//a good spot appeared, check for bad again
                if (checkPixel(visCvThresh,x,y)) {
                    //set good
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                } else {
                    //set bad
                    good=0;
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            } else { //all the rest are bad
                //set bad
                index = offset+x;
                img->imageData[index] = BAD_PIXEL;
            }
        }
    }
}

/*
 * This function scans from the bottom center of image upward,
 *	checking to see if the width of the robot can progress
 *	any further up the image, while sliding left/right as needed
 */
void robotWidthScan(IplImage* img, int& goalx, int& goaly) {

    //int width = img->width;
    //int height = img->height;
    int center = img->width/2;
    int startx = center - ROBOT_WIDTH/2;
    int endx = startx + ROBOT_WIDTH;
    int y = img->height-PIXEL_SKIP;
    int x = startx;
    int half = ROBOT_WIDTH/2;
    int i;

    // return -1 on failure
    goalx=-1;
    goaly=-1;

    /* 	scan from bottom center of image upward,
    	checking to see if the width of the robot can progress
    	any further up the image, sliding left/right as needed
    */
    for ( ; y >= PIXEL_SKIP ; y-- ) {

        //check left, move right
        for (x=startx; x<img->width-ROBOT_WIDTH-1; x++) {
            if (!checkPixel(img,x,y)|| !checkPixel(img,x+half,y) ) {
                x++;	//slide right
                startx=x;
                endx=x+ROBOT_WIDTH;
            } else {
                startx=x;
                endx=x+ROBOT_WIDTH;
                break;
            }
        }

        for ( i = startx; i<endx; i++) { //scan along width to not cross over boundary
            if (!checkPixel(img,i,y)) {
                break;
            } else {
                goalx = i-half;
                goaly = y;
            }
        }
        if (i>=endx) {
            //success, keep going up
            continue;
        }

        //check right, move left
        for (x=endx; x>0+ROBOT_WIDTH; x--) {
            if (!checkPixel(img,x,y) || !checkPixel(img, x-half, y) ) {
                x--;	//slide left
                startx=x-ROBOT_WIDTH;
                endx=x;
            } else {
                startx=x-ROBOT_WIDTH;
                endx=x;
                break;
            }
        }
        for ( i = endx; i>startx; i--) { //scan along width to not cross over boundary
            if (!checkPixel(img,i,y)) {
                break;
            } else {
                goalx = i+half;
                goaly = y;
            }
        }
        if (i<=startx) {
            //success, keep going up
            continue;
        } else {
            break;		//we dont fit left or right
        }
    }

    //sanity check goal
    if (goalx==-1||goaly==-1) {
        //not good
    } else {
        //found goal
        if (!checkPixel(img,goalx,goaly)) {
            goalx=goaly=-1;	//not good, error in scanning
        } else {

            /* GOAL! */

            //=== debug: show goal ===//
            // visCvPath=640x480 //
            //cvLine(visCvDebug, cvPoint(center,height-2), cvPoint(goalx,goaly), (CV_RGB(0,0,0)), 2, 8, 0);
            //cvLine(visCvDebug, cvPoint(goalx-half,goaly), cvPoint(goalx+half,goaly), (CV_RGB(0,0,0)), 2, 8, 0);
            // visCvPath=320x240 //
            cvLine(visCvDebug, cvPoint(center*2,img->height*2-2), 	cvPoint(goalx*2,goaly*2), 		 (CV_RGB(0,0,0)), 2, 8, 0);
            cvLine(visCvDebug, cvPoint((goalx-half)*2,goaly*2), cvPoint((goalx+half)*2,goaly*2), (CV_RGB(0,0,0)), 2, 8, 0);
            //========================//

        }
    }

    // return goalx,goaly,
    // or set -1 on error

}

/*
 *
 *
 */
void visPlanPath(IplImage* img, int& goalx, int& goaly) {
    int width = img->width;
    int height = img->height;
    int center = width/2;

    unsigned char * temp=(unsigned char *) img->imageData;
    //convert
    for (int i = 0 ; i< width*height; i++) {
        if (img->imageData[i]==0) temp[i]=0;
        else temp[i]=255;
    }
    goaly+=ROBOT_WIDTH;

    //path plan! returns 0 on error
    if (planner.navigate( temp,					// image //int xsize, int ysize, int xstart,int ystart,int &xend, int &yend, int scaredD, int widthRobo, int heightRobo) {
                          width,				// xsize
                          height,				// ysize
                          center,				// xstart
                          height-ROBOT_WIDTH/2,	// ystart
                          goalx,				// xend (return value)
                          goaly,				// yend (return value)
                          30,					// scaredD
                          11,//ROBOT_WIDTH,		// widthRobo
                          11)//ROBOT_WIDTH);	// heightRobo
       ) {
        //success
    } else {
        //fail
        //goalx=goaly=-1;
    }

    //=== debug: show goal ===//
    // visCvPath=640x480 //
    //cvLine(visCvDebug, cvPoint( center	,height-2), cvPoint(goalx,goaly), (CV_RGB(255,255,255)), 2, 8, 0);
    // visCvPath=320x240 //
    cvLine(visCvDebug, cvPoint(center*2,height*2-2), cvPoint(goalx*2,goaly*2), (CV_RGB(255,255,255)), 2, 8, 0);
    //========================//

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
void visSweeperLines(Point2D<int>& goal) {

    Graphics g_path(visCvPath);
    Graphics g_draw(visCvDebug);

    static int pathDanger[NAV_PATH__NUM];
    int curPixelDanger = 0;
    int curPathDanger = 0;

    /* Compute and draw all navigation paths we are considering (and do other actions) */
    {
        for (int pathID=0; pathID<NAV_PATH__NUM; pathID++) {
            // Calculate path parameters
            Point2D<double> pathStart = navPath_start(pathID);
            Point2D<double> pathEnd = navPath_end(pathID);

            // Calculate the set of points in the path
            QVector< Point2D<int> > pathPoints;
            Graphics::calculatePointsInLine(
                (int) pathStart.x, (int) pathStart.y,
                (int) pathEnd.x, (int) pathEnd.y,			// @ 640x480: 	maxPathEnd.x=607
                &pathPoints);								//				minPathEnd.x=112


            // Calculate the danger value for the path
            // along with the danger contributions of all
            // the pixels in the path
            curPathDanger = 0;
            uchar pixelDanger[pathPoints.count()];
            for (uint j=0, n2=pathPoints.count(); j<n2; j++) {
                //Point2D<int> curPathPoint = pathPoints[j];

                // Calculate the danger contribution of the current
                // path-pixel to the path danger
                curPixelDanger = 0;
                //int passedwhiteline=0;
                for (int delta=-NAV_PATH__PATH_SEARCH_GIRTH; delta<=NAV_PATH__PATH_SEARCH_GIRTH; delta++) {
                    // (XXX: Consider *nearby* pixels as different fragments of the path-pixel)
                    Point2D<int> curPoint = pathPoints[j];
                    curPoint.x += delta;

                    // check path image (half size) for black=bad
                    if (visCvPath->imageData[curPoint.y/2*visCvPath->width+curPoint.x/2]==0) {
                        // everything bad is a barrel
                        curPixelDanger += DANGER_PER_BARREL_PIXEL;
                    } else {
                        // Nothing special: Probably grass
                        //curPixelDanger += 0;
                    }

                }

                // Compensate for considering *nearby* pixels
                curPixelDanger /= ((NAV_PATH__PATH_SEARCH_GIRTH*2) + 1);
                pixelDanger[j] = curPixelDanger;
                curPathDanger += curPixelDanger;
            }

            //==== weight outer path lines more scary than inner ==========//
            int weight = abs(NAV_PATH__CENTER_PATH_ID-pathID)/(NAV_PATH__NUM);
            curPathDanger += weight;

            // Clip high danger values to be no higher than MAX_PATH_DANGER
            if (curPathDanger > MAX_PATH_DANGER) {
                curPathDanger = MAX_PATH_DANGER;
            }
            pathDanger[pathID] = curPathDanger;

            //if (PRINT_DANGER_VALUES) {
            //	printf("PATH #%d: %d\n", (int) pathID, (int) curPathDanger);
            //}

            // Draw the body of the path using a color that is determined from the path's danger value
            g_draw.setColor(navPath_color(curPathDanger));
            g_draw.drawLine(
                (int) pathStart.x, (int) pathStart.y,
                (int) pathEnd.x, (int) pathEnd.y);

            g_draw.setColor(DANGEROUS_PIXEL_COLOR);
            // Hilight the "dangerous" pixels in the path
            // (that contributed to the path's total danger value)
            for (uint j=0, n2=pathPoints.count(); j<n2; j++) {
                uchar curPixelDanger = pixelDanger[j];
                if (curPixelDanger != 0) {
                    Point2D<int> curPoint = pathPoints[j];

                    // Color the pixel either thickly/thinly,
                    // depending on how dangerous it is
                    if (curPixelDanger > DANGER_PER_BARREL_PIXEL) {	// maximum danger
                        g_draw.drawRect_rational(
                            curPoint.x, curPoint.y,
                            2, 2);
                    } else {
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
        static int smoothedPathDangers[NAV_PATH__NUM];

        // Copy first edge
        for (int curPath_id = 0; curPath_id < NAV_PATH__DANGER_SMOOTHING__RADIUS; curPath_id++) {
            smoothedPathDangers[curPath_id] = pathDanger[curPath_id];
        }

        // Smooth interior
        int sumOfNearbyDangers = 0;
        int avgOfNearbyDangers;
        for (int curPath_id = NAV_PATH__DANGER_SMOOTHING__RADIUS;
                curPath_id < (NAV_PATH__NUM - NAV_PATH__DANGER_SMOOTHING__RADIUS + 1);
                curPath_id++) {
            sumOfNearbyDangers = 0;
            for (int delta = -NAV_PATH__DANGER_SMOOTHING__RADIUS;
                    delta < NAV_PATH__DANGER_SMOOTHING__RADIUS;
                    delta++) {
                sumOfNearbyDangers += pathDanger[curPath_id + delta];
            }
            avgOfNearbyDangers = sumOfNearbyDangers / (NAV_PATH__DANGER_SMOOTHING__RADIUS*2 + 1);

            smoothedPathDangers[curPath_id] = avgOfNearbyDangers;
        }

        // Copy second edge
        for (int curPath_id = (NAV_PATH__NUM - NAV_PATH__DANGER_SMOOTHING__RADIUS);
                curPath_id < NAV_PATH__NUM;
                curPath_id++) {
            smoothedPathDangers[curPath_id] = pathDanger[curPath_id];
        }

        // Transfer smoothed dangers back to primary danger buffer
        for (int curPath_id=0; curPath_id<NAV_PATH__NUM; curPath_id++) {
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
            int bestPath_distanceFromCenter = abs(NAV_PATH__CENTER_PATH_ID - bestPath_id);
            int curPath_danger;
            int curPath_distanceFromCenter;
            for (int curPath_id=0; curPath_id<NAV_PATH__NUM; curPath_id++) {
                curPath_danger = pathDanger[curPath_id];
                curPath_distanceFromCenter = abs(NAV_PATH__CENTER_PATH_ID - curPath_id);

                if (curPath_danger < bestPath_danger) {
                    bestPath_danger = curPath_danger;
                    bestPath_id = curPath_id;
                    bestPath_distanceFromCenter = curPath_distanceFromCenter;
                } else if (curPath_danger == bestPath_danger) {
                    if (curPath_distanceFromCenter < bestPath_distanceFromCenter) {
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
            for (int deltaX = -1; deltaX <= 1; deltaX++) {
                g_draw.drawLine(
                    ((int) bestPath_start.x) + deltaX, (int) bestPath_start.y,
                    ((int) bestPath_end.x) + deltaX, (int) bestPath_end.y);
            }

            /* Update goal
             * (convert to 320x240 frame) */
            goal.x = (int)bestPath_end.x/2;
            goal.y = (int)bestPath_end.y/2;
            //printf("goal(%d,%d) \n",goal.x,goal.y);


        }



        // drive motors
        // display motor output

    }

}


// Converts a measurement in degrees to radians.
double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

Point2D<double> navPath_start(int pathID) {
    return Point2D<double>(
               (visCvRaw->width / 2) + (NAV_PATH__CENTER_PATH_ID-pathID),
               visCvRaw->height - PIXEL_SKIP -1
           );
}

Point2D<double> navPath_vector(int pathID) {
    double deg = navPath_angle(pathID);
    double rad = deg2rad(deg);

    double radius = 0.75 * ((double) visCvRaw->width) / 2;
    return
        Point2D<double>(
            radius*cos(rad),
            -radius*sin(rad))	// computer y-axis is inverse of math y-axis
        * NAV_PATH__VIEW_DISTANCE_MULTIPLIER;
}

Point2D<double> navPath_end(int pathID) {
    return navPath_start(pathID) + navPath_vector(pathID);
}

// result is in degrees
double navPath_angle(int pathID) {
    return
        NAV_PATH__VIEW_CONE__START_ANGLE +
        (pathID * NAV_PATH__VIEW_CONE__SPACING);
}

CvScalar navPath_color(int pathDanger) {
    return cvScalar(
               0, 0,
               (pathDanger / (double)MAX_PATH_DANGER) * 255);
}



