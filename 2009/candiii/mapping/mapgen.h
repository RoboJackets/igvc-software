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
    //void getFeatures();
    void LoadXMLSettings();
    int maxFeatures;
    int minFeatureDistance;
    IplImage* eig_image;
    IplImage* temp_image;
    int _init_;
    void init();

    /**/
    int getFeatures();
    //CvPoint2D32f* features[2];
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

CvPoint2D32f pts1[4];
CvPoint2D32f pts2[4];



};

#endif // MAPGEN_H
