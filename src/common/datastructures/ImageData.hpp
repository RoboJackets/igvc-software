#ifndef IMAGEDATA_H
#define IMAGEDATA_H


#include <opencv2/opencv.hpp>
#include <common/datastructures/SensorData.hpp>

class ImageData : public SensorData
{
  public:
    ImageData(): SensorData() {}
    ImageData(cv::Mat image): SensorData(), _img(image) {}
    ImageData(cv::Mat image, long long int time_us): SensorData(time_us), _img(image) {}
    ImageData deepCopy() { return ImageData(_img.clone(), getTimeMicroSeconds()); }
    cv::Mat& mat() {return _img;}
    virtual ~ImageData() {}

  private:
    cv::Mat _img;
};

#endif // IMAGEDATA_H
