#include "CameraInfo.h"

CameraInfo::CameraInfo()
{

}

CameraInfo CameraInfo::Bumblebee2_BB2_08S2C_38()
{
  CameraInfo cam;
  cam.FocalLength(3.8*pow(10,-3));
  cam.PixelSideLength(4.65*pow(10,-6));
  cam.PixelsPerColumn(768);
  cam.PixelsPerRow(1024);
  cam.Baseline(12*pow(10,-2));
  return cam;
}

/**
Horizontal Field of View of the Camera
Units: Radians
**/
double CameraInfo::HFOV()
{
  return 2*atan2(_pixelsPerRow*_pixelSideLength/2,_focalLength);
}

/**
Vertical Field of View of the Camera
Units: Radians
**/
double CameraInfo::VFOV()
{
  return 2*atan2(_pixelsPerColumn*_pixelSideLength/2,_focalLength);
}

/**
Under the assumption that each pixel corresponds to an equal angular section of the image, gives the angular height of each pixel
Units: Radians (per Pixel)
**/
double CameraInfo::dPhi()
{
  return VFOV()/_pixelsPerColumn;
}

/**
Under the assumption that each pixel corresponds to an equal angular section of the image, gives the angular width of each pixel
Units: Radians (per Pixel)
**/
double CameraInfo::dTheta()
{
  return HFOV()/_pixelsPerRow;
}

/**
Focal Length of the Camera
Units: Meters
**/
double CameraInfo::FocalLength()
{
  return _focalLength;
}

/**
Number of Pixels per Row; a.k.a number of columns
Units: unitless
**/
int CameraInfo::PixelsPerRow()
{
  return _pixelsPerRow;
}

/**
Number of Pixels per Column; a.k.a number of rows
Units: unitless
**/
int CameraInfo::PixelsPerColumn()
{
  return _pixelsPerColumn;
}

/**
Physical length of the side of pixel. Implies square pixels
Units:Meters
**/
double CameraInfo::PixelSideLength()
{
  return _pixelSideLength;
}

/**
Distance between focal points of the two stereo camera. This attribute should really make the basis of a subclass of cameraInfo specifically
for stereo cameras. If you are reading this, you are now tasked with this responsibility
Units: Meters
**/
double CameraInfo::Baseline()
{
  return _baseline;
}

void CameraInfo::FocalLength(double fl)
{
  _focalLength = fl;
}
void CameraInfo::PixelsPerRow(int ppr)
{
  _pixelsPerRow = ppr;
}

void CameraInfo::PixelsPerColumn(int ppc)
{
  _pixelsPerColumn = ppc;
}

void CameraInfo::PixelSideLength(double len)
{
  _pixelSideLength = len;
}

void CameraInfo::Baseline(double bl)
{
  _baseline = bl;
}

CameraInfo::~CameraInfo()
{
}
