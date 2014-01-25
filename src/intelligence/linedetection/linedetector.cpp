#include "linedetector.h"
#include <common/logger/logger.h>
#include <sstream>

using namespace std;
using namespace cv;
char window_name[16] = "Filter Practice";
char original_window_name[9] = "Original";

//For the erosion/dilation stuff
///\var int erosion_elem
///\brief contains the number corresponding to the element used for erosion
///       2 is what we are currently using (an ellipse)
int erosion_elem = 2;
///\var int erosion_size
///\brief specifies the size of the area to be eroded.
int erosion_size = 2;
int dilation_elem = 2;
int dilation_size = 1;

int const max_elem = 2;
int const max_kernel_size = 21;
///\var Mat src
///\brief contains the original, unprocessed image

///\var Mat dst
///\brief contains the new, processed image that isolates the lines
Mat src, dst;


void Erosion( int, void* );
void Dilation( int, void* );
///
/// \brief LineDetector::LineDetector
///        initiates a new instance of a LineDetector.
///        Requires only the image or video file
/// \param imgFile The image or video file to load from
///

LineDetector::LineDetector(std::string imgFile)
{
    cap = cv::VideoCapture(imgFile);
    if(!loadImage(imgFile))
    {
        stringstream msg;
        msg << "[LineDetector] Failed to load imgFile: " << imgFile;
        Logger::Log(LogLevel::Error, msg.str());
        throw "ERROR: Failed to load imgfile!";
    }
}

///
/// \brief LineDetector::loadImage
///        Loads the image or video from a string.
///        public so that it can be iterated through in case
///        of a video file.
/// \param imgFile String with the location of the image/video file
/// \return a boolean that is true if the file was loaded successfully
///
bool LineDetector::loadImage(std::string imgFile){
    ///Saves imgFile as a class variable
    this->imgFile = imgFile;
    ///Reads the image file to my VideoCapture
    ///True if successful
    bool success = cap.read(src);

    ///<dst is a clone that we will apply the algorithm to
    dst = src.clone();
    return success;
}



void LineDetector::applyAlgorithm(){
    ///Total Average of the pixels in the screen
    ///Used to account for brightness variability
    float totalAvg = getAvg();

    ///Blurs the picture just a little
    GaussianBlur(dst, dst, Size(GAUSSSIZE,GAUSSSIZE),2,0);
    GaussianBlur(src, src, Size(GAUSSSIZE,GAUSSSIZE),2,0);
    ///Separates the pixels into black(not lines) and white (lines)
    blackAndWhite(totalAvg);

    Erosion( 0, 0 );
    Dilation(0,0);

    ///Displays both the original and processed images
    displayImage();
}

///
/// \brief Erosion erodes the white away
///
void Erosion( int, void* )
{
  ///erosion_type is set to ellipse in the LineDetector class
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  ///Apply the erosion operation
  erode( dst, dst, element );
}

///
/// \brief Dilation enhances the white lines
///
void Dilation( int, void* )
{
  ///Set to ellipse in LineDetector
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
///Apply the dilation operation
  dilate( dst, dst, element );

}

///
/// \brief LineDetector::blackAndWhite converts the image into
///        black (not lines) and white (lines)
/// \param totalAvg The average brightness of the picture
///
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
                    (p[2]<tempAvg*1.7)&&(p[1]>tempAvg*1.05)&&(ABSDIFF(p[1], p[2]) <20)){
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

///
/// \brief LineDetector::detectObstacle detects orange and bright white obstacles
/// \param row the row of the top of the obstacle
/// \param col the column of the left of the obstacle
///
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

///
/// \brief LineDetector::displayImage Displays both the original and
///        transformed images
///
void LineDetector::displayImage(){
    //Show transformed image
    imshow(window_name, dst);
    //Show original image in a different window
    imshow(original_window_name, src);
    //Wait slightly
    waitKey(DELAY);
}


///
/// \brief LineDetector::getAvg gets the average of the relevant pixels
/// \return the average as a floating point number
///
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

///
/// \brief LineDetector::blackoutSection turns a section of the image black
/// \param rowl the lower row bound
/// \param rowu the upper row bound
/// \param coll the left column bound
/// \param colu the right column bound
///
void LineDetector::blackoutSection(int rowl, int rowu, int coll, int colu){

    for (int i=rowl;i<=rowu;i++){
        for (int j = coll; j<=colu; j++){
            dst.at<Vec3b>(i,j)[0] = 0;
            dst.at<Vec3b>(i,j)[1] = 0;
            dst.at<Vec3b>(i,j)[2] = 0;
        }
    }
}


