#include "vision/GrassOdometer.h"

GrassOdometer::GrassOdometer(ColorRange limits) : _colors(limits)
{
}

void GrassOdometer::processImage(Mat src)
{

  Mat scores, dst_norm, dst_norm_scaled;
  scores = Mat::zeros( src.size(), CV_32FC1 );
  getHarrisScores(src, scores);
  //zeroOutOfRange();
  //getTopScores();

  //compute corresponding locations
  //decide how to combine them, leave it abstract
}


void GrassOdometer::getHarrisScores(Mat& frame ,Mat& scores, int blockSize, int apertureSize, double k)
{
  Mat frame_gray;
  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  cornerHarris(frame_gray, scores, blockSize, apertureSize, k, BORDER_DEFAULT);
}

void GrassOdometer::zeroOutOfRange(Mat& frame, Mat& scores)
{
  int r,c, nRows, nCols;
  nRows = frame.rows;
  nCols = frame.cols;
  uchar* pixel;

  for(r=0; r < nRows ; r++ )
  {
    for(c=0; c < nCols; c++ )
    {
      pixel = &frame.data[frame.step[0]*r + frame.step[1]* c + 0];
      if (_colors.inRangeBGR(pixel) != true)
      {
        scores.at<float>(r,c) = 0;
      }
    }
  }
}


void getTopValues(Mat& scores)
{

}
