#ifndef CAMERAINFO_H
#define CAMERAINFO_H

#include <math.h>
#include <cmath>

#define deg2rad(a) a/180*M_PI
#define rad2deg(a) a*180/M_PI


class CameraInfo
{
  public:
    //Actually useful functions
    CameraInfo();
    static CameraInfo Bumblebee2_BB2_08S2C_38();
    double HFOV();
    double VFOV();
    //Getters
    double FocalLength();
    int PixelsPerRow();
    int PixelsPerColumn();
    double PixelSideLength();
    double Baseline();
    //Setters
    void FocalLength(double);
    void PixelsPerRow(int);
    void PixelsPerColumn(int);
    void PixelSideLength(double);
    void Baseline(double);
    //Dtor
    virtual ~CameraInfo();

  private:
    double _FocalLength;
    int _PixelsPerRow;
    int _PixelsPerColumn;
    double _PixelSideLength;
    double _Baseline;
};

#endif // CAMERAINFO_H
