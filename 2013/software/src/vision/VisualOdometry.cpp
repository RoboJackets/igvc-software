#include "vision/GrassOdometer.h"
#include "vision/ColorRange.h"
#include "vision/colorRangeFinder.h"
#include "common/FLPriotityQueue.hpp"
#include  "common/Robot.h"


#include "sensors/DataStructures/VisOdomData.hpp"
#include "sensors/DataStructures/IMUData.hpp"
#include "sensors/DataStructures/ImageData.hpp"
#include "sensors/DataStructures/StereoImageData.hpp"

#include "sensors/camera3D/CameraInfo.h"

#include <queue>

#include <iostream>
#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;

//Start of copies function

using namespace std;
using namespace Eigen;
using namespace cv;

int main( int argc, char** argv )
{
  CameraInfo info = CameraInfo::Bumblebee2_BB2_08S2C_38();

}
/*
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

int main( int argc, char** argv )
  {
  Mat img_1 = imread( argv[1]);
  Mat img_2 = imread( argv[2]);
  ColorRange limits(4,71,13,144,12,136);
  GrassOdometer odo(limits, 50);
  odo.FuckItWeWillDoItLive(img_1, img_2);

}
*/

/*

void readme();

int main( int argc, char** argv )
{
  if( argc != 3 )
  { readme(); return -1; }

  Mat img_1 = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_2 = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );

  if( !img_1.data || !img_2.data )
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_1, keypoints_2;

  detector.detect( img_1, keypoints_1 );
  detector.detect( img_2, keypoints_2 );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_1, descriptors_2;

  extractor.compute( img_1, keypoints_1, descriptors_1 );
  extractor.compute( img_2, keypoints_2, descriptors_2 );

  //cout << descriptors_1.size() << endl;
  cout << keypoints_1.size() << endl;

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance < 2*min_dist )
    { good_matches.push_back( matches[i]); }
  }

  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Show detected matches
  imshow( "Good Matches", img_matches );

  for( int i = 0; i < good_matches.size(); i++ )
  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

  waitKey(0);


  return 0;


 }

 void readme()
 { std::cout << " Usage: ./SURF_FlannMatcher <img1> <img2>" << std::endl; }

//End of Copied function
*/



/*
using namespace cv;
using namespace std;




int main()
{
  FLPriotityQueue<HarrisScore, greater<HarrisScore> > dat(2);
  dat.push(HarrisScore(1,2,3));
  dat.push(HarrisScore(15,22,32));
  dat.push(HarrisScore(123,222,322));
  FLPriotityQueue<HarrisScore, greater<HarrisScore> > copyDat(2);
  copyDat = dat;
  //cout << "Top before print is " << dat.top();

  HarrisScore thisguy = dat.top();
  cout << "Top before print is " << thisguy;

  cout << dat << endl;

  thisguy = dat.top();

  cout << "Top after print is " << thisguy;
}
*/

/*
int main()
{
  uchar pnt[3];
  pnt[0] = 52;
  pnt[1] = 32;
  pnt[2] = 123;
  ColorRange range(1,255,1,255,1,255);
  ColorRange range2(100,255,1,255,1,255);
  cout << "point is in range? " << range.inRangeBGR(pnt) << endl;
  cout << "point is in range2? " << range2.inRangeBGR(pnt) << endl;
}
*/

/*

int main()
{
    findRange();
    return 0;
}

*/

/*

/// Global variables
Mat src, src_gray;
int thresh = 100;
int max_thresh = 255;

char* source_window = "Source image";
char* corners_window = "Corners detected";
char* grass_window = "Corners on Grass";

/// Function header
void cornerHarris_demo( int, void* );
void grassCorner_demo(int, void* );

// @function main
int main( int argc, char** argv )
{
  /// Load source image and convert it to gray
  string img = "/home/alex/Desktop/BenCode_534_small.jpg";
  src = imread(img, 1 );
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Create a window and a trackbar
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  //createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );
  createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, grassCorner_demo );
  imshow( source_window, src );

  grassCorner_demo(0,0);
  //cornerHarris_demo( 0, 0 );

  waitKey(500000);
  return(0);
}

void grassCorner_demo(int, void*)
{
  ColorRange limits(4,71,13,144,12,136);
  //ColorRange limits(0,255,0,255,0,255);
  GrassOdometer odo(limits);


  Mat dst, dst_norm, dst_norm_scaled, processed_scores, processed_scores_scaled;
  dst = Mat::zeros( src.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );
  ///Create a copy of the scores, zero the ones that aren't on grass
  processed_scores = dst_norm.clone();
  convertScaleAbs( processed_scores, processed_scores_scaled);

  odo.zeroOutOfRange(src, processed_scores);

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
  {
    for( int i = 0; i < dst_norm.cols; i++ )
    {
      if( (int) dst_norm.at<float>(j,i) > thresh )
      {
        circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
      }
      if( (int) processed_scores.at<float>(j,i) > thresh )
      {
        circle(processed_scores_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
      }
    }
  }





  /// Showing the result


  namedWindow( corners_window, CV_WINDOW_AUTOSIZE );
  namedWindow( grass_window, CV_WINDOW_AUTOSIZE );
  imshow(corners_window, dst_norm_scaled );
  imshow(grass_window, processed_scores_scaled);
}


// @function cornerHarris_demo
void cornerHarris_demo( int, void* )
{

  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( src.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
  {
    for( int i = 0; i < dst_norm.cols; i++ )
    {
      if( (int) dst_norm.at<float>(j,i) > thresh )
      {
         circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
      }
    }
  }
  /// Showing the result
  namedWindow( corners_window, CV_WINDOW_AUTOSIZE );
  imshow( corners_window, dst_norm_scaled );
}


*/
