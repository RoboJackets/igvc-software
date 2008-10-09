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


