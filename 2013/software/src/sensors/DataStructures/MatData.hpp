#ifndef MATDATA_H
#define MATDATA_H


#include <opencv2/opencv.hpp>


using namespace cv;
class MatData : SensorData
{
  public:
    MatData(Mat image): _Img(image) {}
    Mat& mat() {return _Img;}
    virtual ~MatData() {}

  private:
    Mat _Img;
};

#endif // MATDATA_H
