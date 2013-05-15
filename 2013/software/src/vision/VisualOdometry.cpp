#include "vision/GrassOdometer.h"
#include "vision/ColorRange.h"
#include "vision/colorRangeFinder.h"

#include "common/FLPriotityQueue.h"

#include <queue>


using namespace cv;
using namespace std;

int main()
{
  FLPriotityQueue<HarrisScore, greater<HarrisScore> > dat(2);
  dat.push(HarrisScore(1,2,3));
  dat.push(HarrisScore(15,22,32));
  dat.push(HarrisScore(123,222,322));

  cout << dat.top().score() << endl;
  cout << dat.size()<<endl;


}


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
