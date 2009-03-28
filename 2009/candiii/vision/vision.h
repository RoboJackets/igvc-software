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

class Vision
{

public:

	Vision();
	~Vision();
	void init();

	// This is called each frame to do ALL vision processing!
	void visProcessFrame(Point2D<int>& goal);

	/* flag for performing image perspective transform */
	int DO_TRANSFORM ;

	/*** SweeperLines ****************************************************/
	// Number of paths that are assessed between the starting/ending angles
	int nav_path__num ;
	// Proportional to the lengths of the paths (in image space)	//0.35;<-with-transform
	double nav_path__view_distance_multiplier ;
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

	// helpers for sweeperlines ////
	void visSweeperLines(Point2D<int>& goal);
	double deg2rad(double degrees);
	Point2D<double> navPath_start(int pathID);
	Point2D<double> navPath_vector(int pathID);
	Point2D<double> navPath_end(int pathID);
	double navPath_angle(int pathID);
	CvScalar navPath_color(int pathDanger);
	/**********************************************************************************/

	// do vision processing using HSV thresholding (processing Method A)
	void visHsvProcessing(Point2D<int>& goal);
	// do vision processing using adaptive thresholding (processing Method B)
	void visAdaptiveProcessing(Point2D<int>& goal);

	// HSV thresholds
	int satThreshold;
	int hueThreshold;

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
	int checkPixel(IplImage* img, const int x, const int y);
	void scanFillLeft(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout);
	void scanFillRight(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout);
	int findBestX(IplImage* img, int height, int center);

	// helpers for path finding
	void robotWidthScan(IplImage* img, Point2D<int>& goal);

	// loads the thresholds for vision processing from the xml config file
	void LoadVisionXMLSettings();

	// trackbar value determines image view to display
	void ConvertAllImageViews(int trackbarVal);

	// manually does color analysis that the HSV colorspace doesn't
	void preProcessColors(IplImage* img);

	// for image display names
	CvFont font;

	// finds min/max values in a 0-255 greyscale image and normalizes using those
	void Normalize(IplImage* img);

	// finds histogram in a 0-255 greyscale image and normalizes using it
	void Equalize(IplImage* img);

	// width of robot in pixels
	int ROBOT_WIDTH;

	/* for the adaptive algorithms */
	void Adapt();
	void CvtPixToGoal(Point2D<int>& goal);
	int adapt_maxDiff;
	int adapt_boxPad;
	int DO_ADAPTIVE;
	CvPoint UL;
	CvPoint LR;
	CvRect roi;
	IplImage* roi_img;
	int avgR;
	int avgG;
	int avgB;

};


#endif // _VISION_H_


