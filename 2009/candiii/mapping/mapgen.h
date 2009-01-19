#ifndef MAPGEN_H
#define MAPGEN_H

#include <cv.h>
#include <highgui.h>
#include "Point2D.h"

/*
 * This file contains the robot's SLAM processing code.
 *   by: Chris McClanahan
 *
 */

class MapGen
{
public:
	MapGen();
	virtual ~MapGen();
public:
	void genMap();
	void getFeatures();
	void LoadXMLSettings();
	int maxFeatures;
	int minFeatureDistance;
	IplImage* eig_image;
	IplImage* temp_image;
	int _init_;
	void init();

};

#endif // MAPGEN_H
