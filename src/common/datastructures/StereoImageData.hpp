#ifndef STEREOIMAGEDATA_H
#define STEREOIMAGEDATA_H

#include <common/datastructures/GPSData.hpp>
#include <common/datastructures/ImageData.hpp>

class StereoImageData : public SensorData
{
  public:
    StereoImageData() {}
    StereoImageData(cv::Mat& left, cv::Mat& right)
    {
      _leftImage = left.clone();
      _rightImage = right.clone();
    }

    ImageData left()
    {
      return ImageData(_leftImage, time());
    }

    ImageData right()
    {
      return ImageData(_leftImage, time());
    }

    cv::Mat& leftMat()
    {
      return _leftImage;
    }

    cv::Mat& rightMat()
    {
      return _rightImage;
    }

    virtual ~StereoImageData() {}
  protected:
  private:
    cv::Mat _leftImage;
    cv::Mat _rightImage;

};

#endif // STEREOIMAGEDATA_H
