#include "linedetector.h"

using namespace std;
using namespace cv;
char window_name[16] = "Filter Practice";
char original_window_name[9] = "Original";

//For the erosion/dilation stuff
int erosion_elem = 2;
int erosion_size = 2;
int dilation_elem = 2;
int dilation_size = 1;
int const max_elem = 2;
int const max_kernel_size = 21;
Mat src, dst;


void Erosion( int, void* );
void Dilation( int, void* );

LineDetector::LineDetector(std::string imgFile)
{
    cap = cv::VideoCapture(imgFile);
    loadImage(imgFile);
    //cout << "\nRows: "<< src.rows<< "\tCols: "<< src.cols <<endl;
}

bool LineDetector::loadImage(std::string imgFile){
    //Save the image file
    this->imgFile = imgFile;

    //Read the image file to my VideoCapture
    bool success = cap.read(src);

    //Make a clone, dst.
    //Dst is what we will apply the algorithm to
    dst = src.clone();
    return success;
}



void LineDetector::applyAlgorithm(){
    //numBlocks - the number of rows and columns to use to divide the screen into blocks
    //for filtering out scatter
    int numBlocks, whiteThreshold;

    //Gets the total average of all the pixels in the screen
    //To account for brightness variability.
    float totalAvg = getAvg();

    //Set to 15 to begin with, may change.
    numBlocks = 15;
    //numBlocks = totalAvg> 90? 60:30;

    //The whiteThreshold varies depending on the average brightness
    whiteThreshold = totalAvg > 90? 5:2;

    //Blur it a little.
    GaussianBlur(dst, dst, Size(GAUSSSIZE,GAUSSSIZE),2,0);
    GaussianBlur(src, src, Size(GAUSSSIZE,GAUSSSIZE),2,0);

    //Separate the pixels into black (not lines) and white (lines)
    blackAndWhite(totalAvg);

    //filter2(numBlocks, whiteThreshold*2);
    //Leave for now, may take out later
    //GaussianBlur(dst, dst, Size(GAUSSSIZE,GAUSSSIZE),2,0);

    //Get rid of the scatter
    //filter2(numBlocks*4, whiteThreshold*2);
    //filter2(numBlocks, whiteThreshold);


    //Erosion/Dilation practice stuff below
    //fpt erosionPTR = (fpt) Erosion;
      Erosion( 0, 0 );
      Dilation(0,0);

    displayImage();
}

void Erosion( int, void* )
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  erode( dst, dst, element );
  //imshow( "Erosion Demo", erosion_dst );
}

void Dilation( int, void* )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( dst, dst, element );
}


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
    //NOTE: look into incorporating lidar data for that.
    int tempAvg;
    for (int i = rows/3; i< rows*5/6; i++){
        for(int j=0; j< cols; j++){
            tempAvg = totalAvg*(1.1 - i*.1/768);
            p = dst.at<Vec3b>(i, j); //Current pixel

            //int colorAvg = (p[0]+p[1]+p[2])/3;
            //int maxDiff = AVGDIFF(p[0], p[1], p[2]);
           /* if ((colorAvg>= totalAvg*COLORRATIO)&&(colorAvg<= totalAvg*COLORRATIOMAX)&&(maxDiff < 90)&&
                    (maxDiff > (totalAvg > 90? 30:15 ))&&(i>src.rows*2/6)&&(p[0]>totalAvg*1.7)){*/

            //If there is a significant amount of red in the pixel, it's most likely an orange cone
            //Get rid of the obstacle
            if (p[2] > totalAvg*2|| p[2] > 253){
                detectObstacle(i, j);
            }
            //Filters out the white and makes it pure white
            if((p[0]>tempAvg*1.5)&& (p[0] < tempAvg*2)&& (p[1] < tempAvg*1.6)&&(p[2]>tempAvg*1.1) &&(p[2]<tempAvg*1.7)&&(p[1]>tempAvg*1.05)){
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

void LineDetector::displayImage(){
    //Show transformed image
    imshow(window_name, dst);
    //Show original image in a different window
    imshow(original_window_name, src);
    //Wait slightly
    waitKey(DELAY);
}
//Currently retired
void LineDetector::filter(int numBlocks, int whiteThreshold){
    //Get the number of rows and columns in each block
    int blockRows = src.rows/numBlocks;
    int blockCols = src.cols/numBlocks;
    int myAvg, numNbor;

    //Loop through each block
    for (int i=1; i<numBlocks-1; i++){
        for(int j=1; j<numBlocks-1;j++){
            //Get block average color
            myAvg+= getBlockAvg(i*blockRows, (i+1)*blockRows-1, j*blockCols, (j+1)*blockCols-1);
            if ((myAvg > whiteThreshold)){//If the average is higher than the Threshol
                numNbor = checkNbors(i, j, blockRows, blockCols, whiteThreshold);
                if (numNbor<2||numNbor > 7){
                    blackoutSection(i*blockRows, (i+1)*blockRows-1, j*blockCols, (j+1)*blockCols-1);
                }
            }
            else blackoutSection(i*blockRows, (i+1)*blockRows-1, j*blockCols, (j+1)*blockCols-1);
        }
    }
}
//TODO modify this to only account for the used space (not the top and bottom)
void LineDetector::filter2(int numBlocks, int whiteThreshold){
    //Get the number of rows and columns in each block
    int blockRows = src.rows/numBlocks;
    int blockCols = src.cols/numBlocks;
    int myAvg, numNbor;

    //Loop through the blocks
    for (int i=1; i<numBlocks-1; i++){
        for(int j=1; j<numBlocks-1;j++){
            //Get block average
            myAvg = getBlockAvg(i*blockRows, (i+1)*blockRows-1, j*blockCols, (j+1)*blockCols-1);
            if ((myAvg > whiteThreshold)){//If white enough, check neighbors
                numNbor = checkNbors(i, j, blockRows, blockCols, whiteThreshold*10);
                if (numNbor<3){//If 1 or no neighbors, turn it black
                    blackoutSection(i*blockRows, (i+1)*blockRows-1, j*blockCols, (j+1)*blockCols-1);
                }
            }
            else blackoutSection(i*blockRows, (i+1)*blockRows-1, j*blockCols, (j+1)*blockCols-1);
        }
    }
}


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

void LineDetector::blackoutSection(int rowl, int rowu, int coll, int colu){
//    int channels = dst.channels();
//    coll *=channels;
//    colu *=channels;
//    uchar* p = dst.data;
    for (int i=rowl;i<=rowu;i++){
        for (int j = coll; j<=colu; j++){
            dst.at<Vec3b>(i,j)[0] = 0;
            dst.at<Vec3b>(i,j)[1] = 0;
            dst.at<Vec3b>(i,j)[2] = 0;
//            p[i*dst.cols+j] =0;
//            p[i*dst.cols+j+1] =0;
//            p[i*dst.cols+j+2] =0;
        }
    }
}

int LineDetector::checkNbors(int rowStartNum, int colStartNum, int rowSize, int colSize, int threshold){
    uchar avg =0, numNbors =0;
    for (int i =-1; i<=1; i++){
        for(int j= -1; j<=1; j++){
            avg = getBlockAvg((rowStartNum+j)*rowSize, (rowStartNum+j+1)*rowSize-1, (colStartNum+i)*colSize, (colStartNum+i+1)*colSize-1);
            numNbors+= (avg > threshold)? 1:0;
        }
    }
    return numNbors;
}

int LineDetector::getBlockAvg(int rowl, int rowu, int coll, int colu){
    int avg = 0;
    int num = (rowl-rowu+1)*(coll-colu+1);
    int channels = src.channels();
    coll *= channels;
    colu *=channels;
    uchar* p = src.data;
    for (int i = rowl;i<=rowu; i++){
        for (int j = coll; j<= colu;j++){
            avg += p[i*src.cols+j];
        }
    }
    return (avg/num);
}
