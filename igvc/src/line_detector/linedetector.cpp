#include "linedetector.h"
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

cv_bridge::CvImagePtr cv_ptr;

void LineDetector::img_callback(const sensor_msgs::ImageConstPtr& msg) {

    cv_ptr = cv_bridge::toCvCopy(msg, "");
     /// Convert to grayscale
//     cvtColor( cv_ptr->image, cv_ptr->image, CV_BGR2GRAY );

     /// Apply Histogram Equalization
//     equalizeHist( cv_ptr->image, cv_ptr->image );

    GaussianBlur(cv_ptr->image, cv_ptr->image, Size(11, 11), 0);

    typedef Vec<unsigned char, 3> pixel;
    Mat imgmat = cv_ptr->image;
//    auto it = imgmat.begin<pixel>();

    Vec3b p;
    int rows = imgmat.rows;
    int cols = imgmat.cols;

    int grey;
    //Turn the top quarter of the screen and bottom sixth of the screen black
    //We can disregard these areas - may extend the bottom of the screen slightly later on
    for (int i = 0; i< rows; i++){
        for(int j=0; j< cols; j++) {
            grey = imgmat.at<Vec3b>(i, j)[0];
            if (grey < 200) {
                imgmat.at<Vec3b>(i, j)[0] = 0;
                imgmat.at<Vec3b>(i, j)[1] = 0;
                imgmat.at<Vec3b>(i, j)[2] = 0;
            } else {
                imgmat.at<Vec3b>(i, j)[1] = grey;
                imgmat.at<Vec3b>(i, j)[2] = grey;
            }
        }
    }

    Erosion(&imgmat);
//    while(it != imgmat.end<pixel>()) {
//        pixel p = *it;
//        p[2] = 0;
//        it++;
//    }
//    Mat kernal = (Mat_<double>(3, 3) << 1, 0, 0, 0, 0, 0, -1);
//    kernal = kernal/9;
//    filter2D(cv_ptr->image, cv_ptr->image, -1, kernal);


    cv_ptr->image = imgmat;
    _filt_img.publish(cv_ptr->toImageMsg());
//    _filt_img.publish(imgmat);
}

//void LineDetector::FindLines(CvImagePtr cv_ptr) {


//}

LineDetector::LineDetector(ros::NodeHandle &handle)
    : max_elem(2),
      max_kernel_size(2),
      gaussian_size(7),
	  _it(handle)
{
    erosion_elem = 2;
    erosion_size = 1;
    dilation_elem = 2;
    dilation_size = 2;

    _src_img = _it.subscribe("/stereo/left/image_raw", 1, &LineDetector::img_callback, this);
	_filt_img = _it.advertise("/filt_img", 1);
}

bool LineDetector::isWorking() {
    return true;
}

void LineDetector::onImageEvent() {
	

    src = cv_ptr->image;
    dst = src.clone();
    cv::resize(dst, dst, cv::Size(512, 384));
    /** Total Average of the pixels in the screen. Used to account for brightness variability. */
    float totalAvg = getAvg();

    /** Blurs the picture just a little */
    GaussianBlur(dst, dst, Size(gaussian_size,gaussian_size),2,0);
    /** Separates the pixels into black(not lines) and white (lines) */
    blackAndWhite(totalAvg);

//    Erosion();
    Dilation();
    cv::Mat transformDst(dst.rows, dst.cols, CV_8UC3);
    //transformPoints(dst, transformDst);
	// TODO FIND WHAT PARAM TO PUT IN PUBLISH
    //_filt_img.publish(dst);
    //cloud = toPointCloud(transformDst);

	// TODO REPLACE WITH ROS COMMUNICATION
    //onNewLines(ImageData(dst));
    //onNewLinesMat(dst);
    cout <<"Sending new matrix"<<endl;
    pcl::PointXY offset;
	// TODO FIND REPLACEMENT FOR CONFIGMANAGER
    //offset.x = ConfigManager::Instance().getValue("Camera", "OffsetX", 0.0f);
    //offset.y = ConfigManager::Instance().getValue("Camera", "OffsetY", 0.0f);

//    int timeElapsed = t.msecsTo(QDateTime::currentDateTime().time());
//    cout << "Time elapsed: " << timeElapsed <<endl;

	// TODO REPLACE WITH ROS COMMUNICATION
    //onNewCloud(cloud, offset);


//    timeElapsed = t.msecsTo(QDateTime::currentDateTime().time());
//    cout << "Time elapsed: " << timeElapsed <<endl;
}

void LineDetector::Erosion(Mat* dst) {
  int erosion_type = MORPH_ELLIPSE;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  // Apply the erosion operation
  erode( *dst, *dst, element );
}

/** Dilation enhances the white lines */
void LineDetector::Dilation()
{
  int dilation_type = MORPH_ELLIPSE;
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
    int rows = dst.rows;
    int cols = dst.cols;

    //Turn the top quarter of the screen and bottom sixth of the screen black
    //We can disregard these areas - may extend the bottom of the screen slightly later on
    for (int i = 0; i< rows*4/9; i++){
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
	// TODO FIND REPLACEMENT FOR CONFIGMANAGER
    /*float redUp = ConfigManager::Instance().getValue("LineDetector", "RedUp", 1.7);
    float redDown = ConfigManager::Instance().getValue("LineDetector", "RedDown", 1);
    float greenUp = ConfigManager::Instance().getValue("LineDetector", "GreenUp", 1.7);
    float greenDown = ConfigManager::Instance().getValue("LineDetector", "GreenDown", .8);
    float blueUp = ConfigManager::Instance().getValue("LineDetector", "BlueUp", 2.3);
    float blueDown = ConfigManager::Instance().getValue("LineDetector", "BlueDown", 0);
    int diff = ConfigManager::Instance().getValue("LineDetector", "diff", 5);
    for (int i = rows*4/9; i< rows*5/6; i++){
        for(int j=0; j< cols; j++){
            tempAvg = totalAvg*(1.1 - i*.1/768);
            p = dst.at<Vec3b>(i, j); //Current pixel

            //If there is a significant amount of red in the pixel, it's most likely an orange cone
            //Get rid of the obstacle
            if (*//*p[2] > totalAvg*2 && *//*p[2] > 253){
                detectObstacle(i, j);
            }*/

            //Filters out the white and makes it pure white
            /*if((p[0]>tempAvg*blueDown) && (p[0] < tempAvg*blueUp) || (p[0] < 20 && p[1] < 20 && p[2] < 20) *///*&& (p[1] < tempAvg*greenUp) && (p[2]>tempAvg*redDown)*/
                    /*&& (p[2]<tempAvg*redUp) && (p[1]>tempAvg*greenDown)*//* && (abs(p[1] - p[2]) <tempAvg/diff)*//*) {
                dst.at<Vec3b>(i,j)[0] = 0;
                dst.at<Vec3b>(i,j)[1] = 0;
                dst.at<Vec3b>(i,j)[2] = 0;

            }

            else { //Otherwise, set pixel to black
                dst.at<Vec3b>(i,j)[0] = 255;
                dst.at<Vec3b>(i,j)[1] = 255;
                dst.at<Vec3b>(i,j)[2] = 255;//all 0's
            }
        }
    }*/
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
    Mat region = dst(Range(dst.rows/6, 5*dst.rows/6), Range(dst.cols/6, 5*dst.cols/6));
    Scalar sumScalar = cv::sum(region);
    float avg = sumScalar[0] + sumScalar[1] + sumScalar[2];
    avg /= dst.rows * dst.cols * dst.channels();
    return avg;
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


