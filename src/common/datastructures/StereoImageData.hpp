#ifndef STEREOIMAGEDATA_H
#define STEREOIMAGEDATA_H

#include <common/datastructures/GPSData.hpp>
#include <common/datastructures/ImageData.hpp>

class StereoImageData : public SensorData
{
  public:
    StereoImageData() {}
    StereoImageData(Mat& left, Mat& right)
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

    Mat& leftMat()
    {
      return _leftImage;
    }

    Mat& rightMat()
    {
      return _rightImage;
    }

    virtual ~StereoImageData() {}
  protected:
  private:
    Mat _leftImage;
    Mat _rightImage;

};

#endif // STEREOIMAGEDATA_H
