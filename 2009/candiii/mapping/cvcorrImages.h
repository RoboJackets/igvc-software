/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

//#include "_cvaux.h"
//#include "cvtypes.h"
//#include <float.h>
//#include <limits.h>
//#include "cv.h"
//#include "highgui.h"

#include <cv.h>
#include <highgui.h>
#include <stdio.h>

/* Valery Mosyagin */

/* ===== Function for find corresponding between images ===== */

/* Create feature points on image and return number of them. Array points fills by found points */
int icvCreateFeaturePoints(IplImage *image, CvMat *points, CvMat *status);
/*-------------------------------------------------------------------------------------*/

/* For given points1 (with pntStatus) on image1 finds corresponding points2 on image2 and set pntStatus2 for them */
/* Returns number of corresponding points */
int icvFindCorrForGivenPoints( IplImage *image1,/* Image 1 */
							   IplImage *image2,/* Image 2 */
							   CvMat *points1,
							   CvMat *pntStatus1,
							   CvMat *points2,
							   CvMat *pntStatus2,
							   int useFilter,/*Use fundamental matrix to filter points */
							   double threshold);/* Threshold for good points in filter */
/*-------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------*/
int icvRemoveDoublePoins(   CvMat *oldPoints,/* Points on prev image */
							CvMat *newPoints,/* New points */
							CvMat *oldStatus,/* Status for old points */
							CvMat *newStatus,
							CvMat *origStatus,
							float threshold);/* Status for new points */


void icvComputeProjectMatrix(CvMat* objPoints,CvMat* projPoints,CvMat* projMatr);

/*-------------------------------------------------------------------------------------*/
void icvComputeProjectMatrixStatus(CvMat *objPoints4D,CvMat *points2,CvMat *status, CvMat *projMatr);


void icvAddNewImageToPrevious____(
	IplImage *newImage,//Image to add
	IplImage *oldImage,//Previous image
	CvMat *oldPoints,// previous 2D points on prev image (some points may be not visible)
	CvMat *oldPntStatus,//Status for each point on prev image
	CvMat *objPoints4D,//prev 4D points
	CvMat *newPoints,  //Points on new image corr for prev
	CvMat *newPntStatus,// New point status for new image
	CvMat *newFPoints2D1,//new feature points on prev image
	CvMat *newFPoints2D2,//new feature points on new image
	CvMat *newFPointsStatus,
	CvMat *newProjMatr,
	int useFilter,
	double threshold);//New projection matrix

/*-------------------------------------------------------------------------------------*/
int icvDeleteSparsInPoints(  int numImages,
							 CvMat **points,
							 CvMat **status,
							 CvMat *wasStatus);/* status of previous configuration */

