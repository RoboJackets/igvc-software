#ifndef COLORRANGE_H
#define COLORRANGE_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


class ColorRange
{
  public:

    ColorRange(int bMin, int bMax, int gMin, int gMax, int rMin, int rMax);
    bool inRangeBGR(uchar* pixel);
    virtual ~ColorRange();
  protected:
  private:
    int _bMax;
    int _bMin;
    int _gMax;
    int _gMin;
    int _rMax;
    int _rMin;
};

#endif // COLORRANGE_H
