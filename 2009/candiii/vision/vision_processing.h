#ifndef VISION_PROCESSING_H_
#define VISION_PROCESSING_H_


#include <cv.h>
#include <highgui.h>
#include "Point2D.h"


/*
 * This file should contain most of the vision processing algorithms, to be used in vision.cc
 *   by: Chris McClanahan
 *
 * Only IplImages should be used in this file!
 */

#define DO_TRANSFORM 1

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






// splits raw image into RGB channel images
void GetRGBChannels();
// splits raw image into HSV channel images
void GetHSVChannels();
// binary thresholds an image
void ThresholdImage(IplImage *src, IplImage *dst, int thresh);

// analyze visCvThresh and generate visCvPath
void visGenPath(IplImage* img);

// helpers for visGenPath
int checkPixel(IplImage* img, int x, int y);
void scanFillLeft(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout);
void scanFillRight(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout);
int findBestX(IplImage* img, int height, int center);

// helpers for path finding
void robotWidthScan(IplImage* img, int& goalx, int& goaly);
void visPlanPath(IplImage* img, int& goalx, int& goaly);

// for sweeper lines ////
void visSweeperLines(Point2D<int>& goal);
double deg2rad(double degrees);
Point2D<double> navPath_start(int pathID);
Point2D<double> navPath_vector(int pathID);
Point2D<double> navPath_end(int pathID);
double navPath_angle(int pathID);
// interpolates color
CvScalar navPath_color(int pathDanger);
// end for sweeper lines ////

#endif //define VISION_PROCESSING_H_


