#ifndef curtain_h
#define curtain_h


//#include "Pixel.h"
#include "stdio.h"
#include "image.h"
#include "Buffer2D.h"
#include "PixelRGB.h"

typedef  Buffer2D<PixelRGB> b2drgb;
typedef  Buffer2D<bool> b2dbool;

Buffer2D<bool>& curtain (Buffer2D<PixelRGB>& whim, Buffer2D<PixelRGB>& orim);
void dilate1D (Buffer2D<bool>& arr);
Buffer2D<bool>& cutout(int idx,Buffer2D<bool>& img);



#endif
