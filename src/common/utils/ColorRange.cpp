#include "ColorRange.h"

ColorRange::ColorRange(int bMin, int bMax, int gMin, int gMax, int rMin, int rMax)
{
  _bMin = bMin;
  _gMin = gMin;
  _rMin = rMin;
  _bMax = bMax;
  _gMax = gMax;
  _rMax = rMax;
}

bool ColorRange::inRangeBGR(uchar* pixel)
{
  int r,g,b;
  b = pixel[0];
  g = pixel[1];
  r = pixel[2];
  //std::cout << "blue " << b << ", green " << g << ",red " << r << "." << std::endl;
  if (((_rMin <= r) && (r <= _rMax)) && ((_gMin <= g) && (g <= _gMax)) && ((_bMin <= b) && (b <= _bMax)))
  {
    //std::cout << "point in range" << std::endl;
    return true;
  }
  else
  {

    return false;
  }
}


ColorRange::~ColorRange()
{
  //dtor
}
