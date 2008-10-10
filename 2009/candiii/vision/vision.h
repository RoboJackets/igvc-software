#ifndef _VISION_H_
#define _VISION_H_



#include <cv.h>
#include <highgui.h>
#include "Point2D.h"


/*
 * This file contains the robot's primary vision processing code main function.
 *   by: Chris McClanahan
 *
 */

/* flag for performing image perspective transform */
#define DO_TRANSFORM 1


class Vision {

public:
	Vision();
	~Vision();
	void init();

/*** SweeperLines ****************************************************/
#if DO_TRANSFORM
// Number of paths that are assessed between the starting/ending angles
 int nav_path__num ;
// Proportional to the lengths of the paths (in image space)	//0.35;<-with-transform
 double nav_path__view_distance_multiplier ;
#else
// Number of paths that are assessed between the starting/ending angles
 int nav_path__num ;
// Proportional to the lengths of the paths (in image space)	//0.35;<-with-transform
 double nav_path__view_distance_multiplier ;
#endif

// Defines the "view/navigation cone", which is where the set of
// considered navigation paths is taken from.
 double nav_path__view_cone__offset ;
 double nav_path__view_cone__start_angle ;
 double nav_path__view_cone__end_angle  ;
 double nav_path__view_cone__delta_angle ;
 double nav_path__view_cone__spacing  ;

// XXX: Controls how many pixels *near* a path-pixel are searched for dangerous pixels
// 0 <= nav_path__path_search_girth
 int nav_path__path_search_girth ;
// (do not change without reason!)
 int nav_path__center_path_id ;

// Amount of danger posed by a single barrel-pixel
// (everything bad is a barrel)
 int danger_per_barrel_pixel ;
// Path danger values higher than this will be clipped to this value
 int max_path_danger ;			// >= 0
 int nav_path__danger_smoothing_radius ;
// colors
 CvScalar min_path_danger_color ;
 CvScalar max_path_danger_color ;
 CvScalar dangerous_pixel_color ;

/**********************************************************************************/




// "path planner"
//PathPlan planner; // the navigation class object

int satThreshold;
int hueThreshold;
//int goalx,goaly;
//Point2D<int> goal;
// goal set by robotWidthScan()
Point2D<int> goal_far;
// goal set by visSweeperLines()
Point2D<int> goal_near;



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






// This is called each frame to do vision processing
void visProcessFrame(Point2D<int>& goal);

// loads the thresholds for vision processing from the xml config file
void LoadVisionXML();

// trackbar value determines image view to display
void ConvertAllImageViews(int trackbarVal);





};


#endif // _VISION_H_

