#include "linedetector.h"
#include <common/logger/logger.h>
#include <sstream>
#include <iostream>
#include <cstdlib>

using namespace std;
using namespace cv;

LineDetector::LineDetector(Event<ImageData> &evtSrc)
    : LonImageEvent(this),
      max_elem(2),
      max_kernel_size(2),
      gaussian_size(7)
{
    evtSrc+= &LonImageEvent;
    erosion_elem = 2;
    erosion_size = 2;
    dilation_elem = 2;
    dilation_size = 1;
}

void LineDetector::onImageEvent(ImageData imgd){
    src = imgd.mat();
    dst = src.clone();
    /** Total Average of the pixels in the screen. Used to account for brightness variability. */
    float totalAvg = getAvg();

    /** Blurs the picture just a little */
    GaussianBlur(dst, dst, Size(gaussian_size,gaussian_size),2,0);
    GaussianBlur(src, src, Size(gaussian_size,gaussian_size),2,0);
    /** Separates the pixels into black(not lines) and white (lines) */
    blackAndWhite(totalAvg);

    Erosion();
    Dilation();

    onNewLines(ImageData(dst));
}

void LineDetector::Erosion()
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  // Apply the erosion operation
  erode( dst, dst, element );
}

/** Dilation enhances the white lines */
void LineDetector::Dilation()
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  // Apply the dilation operation
  dilate( dst, dst, element );

}

/**
 *  @brief LineDetector::blackAndWhite converts the image into
 *         black (not lines) and white (lines)
 *  @param totalAvg The average brightness of the picture
 */
void LineDetector::blackAndWhite(float totalAvg){
    Vec3b p;
    int rows = src.rows;
    int cols = src.cols;

    //Turn the top quarter of the screen and bottom sixth of the screen black
    //We can disregard these areas - may extend the bottom of the screen slightly later on
    for (int i = 0; i< rows/3; i++){
        for(int j=0; j< cols; j++){
             dst.at<Vec3b>(i,j)[0] = 0;
             dst.at<Vec3b>(i,j)[1] = 0;
             dst.at<Vec3b>(i,j)[2] = 0;
        }
    }
    for (int i = rows*5/6; i< rows; i++){
        for(int j=0; j< cols; j++){
             dst.at<Vec3b>(i,j)[0] = 0;
             dst.at<Vec3b>(i,j)[1] = 0;
             dst.at<Vec3b>(i,j)[2] = 0;
        }
    }

    //Loops through relevant parts of the image and scans for white lines
    //Also tries to detect obstacles
    int tempAvg;
    for (int i = rows/3; i< rows*5/6; i++){
        for(int j=0; j< cols; j++){
            tempAvg = totalAvg*(1.1 - i*.1/768);
            p = dst.at<Vec3b>(i, j); //Current pixel

            //If there is a significant amount of red in the pixel, it's most likely an orange cone
            //Get rid of the obstacle
            if (p[2] > totalAvg*2|| p[2] > 253){
                detectObstacle(i, j);
            }
            //Filters out the white and makes it pure white
            if((p[0]>tempAvg*1.5)&& (p[0] < tempAvg*2.2)&& (p[1] < tempAvg*1.6)&&(p[2]>tempAvg*1.1) &&
                    (p[2]<tempAvg*1.7)&&(p[1]>tempAvg*1.05)&&(abs(p[1] - p[2]) <20)){
                dst.at<Vec3b>(i,j)[0] = 255;
                dst.at<Vec3b>(i,j)[1] = 255;
                dst.at<Vec3b>(i,j)[2] = 255;

            }
            else { //Otherwise, set pixel to black
                dst.at<Vec3b>(i,j)[0] = 0;
                dst.at<Vec3b>(i,j)[1] = 0;
                dst.at<Vec3b>(i,j)[2] = 0;
            }
        }
    }
}

/**
 *  \brief LineDetector::detectObstacle detects orange and bright white obstacles
 *  \param col the column of the left of the obstacle
 */
void LineDetector::detectObstacle(int row, int col){
    Vec3b p = dst.at<Vec3b>(row,col);
    int row2 = row;
    int col2 = col;

    //While the pixel is still orange, turn it black
    //Then on to the next one, by row
    while (p[2]>100){
        dst.at<Vec3b>(row2, col)[0] = 0;
        dst.at<Vec3b>(row2, col)[1] = 0;
        dst.at<Vec3b>(row2, col)[2] = 0;
        p = dst.at<Vec3b>(++row2, col);
    }
    p = dst.at<Vec3b>(row,col);

    //While the pixel is still orange, turn it black
    //Then on to the next one, by column
    while (p[2]>100){
        dst.at<Vec3b>(row, col2)[0] = 0;
        dst.at<Vec3b>(row, col2)[1] = 0;
        dst.at<Vec3b>(row, col2)[2] = 0;
        p = dst.at<Vec3b>(row, ++col2);
    }

    //Turn everything in that block we just found black
    for(int i = row+1; i<row2;i++){
        for (int j = col+1; j<col2; j++){
            dst.at<Vec3b>(i,j)[0] = 0;
            dst.at<Vec3b>(i,j)[1] = 0;
            dst.at<Vec3b>(i,j)[2] = 0;
        }
    }
}

/**
 *  \brief LineDetector::getAvg gets the average of the relevant pixels
 *  \return the average as a floating point number
 */
float LineDetector::getAvg(){
    Vec3b p;
    float totalAvg = 0;
    for (int i = src.rows/3; i< 5*src.rows/6; i++){
        for(int j=src.cols/6; j< 5*src.cols/6; j++){
            p = dst.at<Vec3b>(i, j);
            totalAvg += (p[0]+p[1]+p[2])/3;
        }
    }
    totalAvg = (25*totalAvg)/(src.cols*src.rows*8);
    return totalAvg;
}

/**
 *  \brief LineDetector::blackoutSection turns a section of the image black
 *  \param rowl the lower row bound
 *  \param rowu the upper row bound
 *  \param coll the left column bound
 *  \param colu the right column bound
 */
void LineDetector::blackoutSection(int rowl, int rowu, int coll, int colu){

    for (int i=rowl;i<=rowu;i++){
        for (int j = coll; j<=colu; j++){
            dst.at<Vec3b>(i,j)[0] = 0;
            dst.at<Vec3b>(i,j)[1] = 0;
            dst.at<Vec3b>(i,j)[2] = 0;
        }
    }
}


