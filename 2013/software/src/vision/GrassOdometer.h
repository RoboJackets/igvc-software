#ifndef GRASSODOMETER_H
#define GRASSODOMETER_H

#include "vision/ColorRange.h"

#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class GrassOdometer
{
  public:
    GrassOdometer(ColorRange limits);
    void processImage(Mat src);
    void getHarrisScores(Mat& frame ,Mat& scores, int blockSize = 2, int apertureSize = 3, double k = 0.04);
    void zeroOutOfRange(Mat& frame, Mat& scores);
  private:

    ColorRange _colors;
};

#endif //GRASSODOMETER_H
