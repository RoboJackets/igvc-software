#ifndef IMAGEDATA_H
#define IMAGEDATA_H


#include <opencv2/opencv.hpp>
#include <common/datastructures/SensorData.hpp>

using namespace cv;

class ImageData : public SensorData
{
  public:
    ImageData() {}
    ImageData(Mat image): SensorData(), _img(image) {}
    ImageData(Mat image, double time): SensorData(time),_img(image) {}
    ImageData deepCopy() { return ImageData(_img.clone(),time()); }
    Mat& mat() {return _img;}
    virtual ~ImageData() {}



  private:
    Mat _img;
};

#endif // IMAGEDATA_H
