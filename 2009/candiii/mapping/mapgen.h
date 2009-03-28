#ifndef MAPGEN_H
#define MAPGEN_H

#include <cv.h>
#include <highgui.h>
#include "Point2D.h"
#include "cvcorrImages.h"
#include <vector>

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

	int genMap();
	void LoadXMLSettings();
	int maxFeatures;
	void init();
	/**/
	int getFeatures();
	CvMat* points1;
	CvMat* points2;
	CvMat* status1;
	CvMat* status2;
	IplImage* prev;
	int processFeatures();
	/**/
	CvMat* matCamToCam;
	CvMat* matCamToWorld;
	IplImage* worldmap;
	void printCv33Matrix(CvMat* matrix);
	int numFramesBack;
	CvPoint2D32f pts1[3];
	CvPoint2D32f pts2[3];
	int yFeatureThresh;
	/**/
	std::vector< std::pair<CvPoint2D32f,CvPoint2D32f> > matchList;
	int maxFeatureShift;
	int imgHalfHeight;
	int imgHalfWidth;
	int genLandmarkMap();
	void mapCamPointToWorldPoint(CvPoint2D32f& cam, CvPoint2D32f& world);
	void mapCamPointToWorldPoint(double camx, double camy, double& worldx, double& worldy);
	std::vector< CvPoint3D32f > worldPoints; // z is ref count
	void addOrUpdateWorldPoint(CvPoint3D32f wpt);
	/**/
	int genProbabilityMap();
	IplImage* probmap;
	int processMap();
	CvPoint robotBaseAt;    // bottom of current image being pasted into world (in world coordinates)
	CvPoint robotLookingAt; // top of current image being pasted into world (in world coordinates)


};

#endif // MAPGEN_H
