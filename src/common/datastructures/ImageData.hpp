#ifndef IMAGEDATA_H
#define IMAGEDATA_H


#include <opencv2/opencv.hpp>
#include <common/datastructures/SensorData.hpp>

class ImageData : public SensorData
{
  public:
    ImageData() {}
    ImageData(cv::Mat image): SensorData(), _img(image) {}
    ImageData(cv::Mat image, double time): SensorData(time),_img(image) {}
    ImageData deepCopy() { return ImageData(_img.clone(),time()); }
    cv::Mat& mat() {return _img;}
    virtual ~ImageData() {}



  private:
    cv::Mat _img;
};

#endif // IMAGEDATA_H
