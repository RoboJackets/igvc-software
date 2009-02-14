#ifndef MAPGEN_H
#define MAPGEN_H

#include <cv.h>
#include <highgui.h>
#include "Point2D.h"
#include "cvcorrImages.h"

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
    void LoadXMLSettings();
    int maxFeatures;
    int _init_;
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
    CvMat* mat_CamToCam;
    CvMat* mat_CamToWorld;
    IplImage* worldmap;
    void printMatrix(CvMat* matrix);
    int numFramesBack;
    CvPoint2D32f pts1[3];
    CvPoint2D32f pts2[3];
    /**/
    int computeHomography(CvPoint2D32f* p1, CvPoint2D32f* p2, CvMat* h);


};

#endif // MAPGEN_H
