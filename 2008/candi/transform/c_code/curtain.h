#ifndef curtain_h
#define curtain_h


//#include "Pixel.h"
#include "stdio.h"
#include "image.h"
#include "Buffer2D.h"
#include "PixelRGB.h"

typedef  Buffer2D<PixelRGB> b2drgb;
typedef  Buffer2D<bool> b2dbool;

b2drgb& curtain (Buffer2D<PixelRGB>& whim, Buffer2D<PixelRGB>& orim);
void RGBtoBool (Buffer2D<PixelRGB>& img,Buffer2D<bool>& dst);
void dilate1D (Buffer2D<bool>& arr);
Buffer2D<bool>& cutout(int idx,Buffer2D<bool>& img);
void booltoRGB (Buffer2D<bool>& img, Buffer2D<PixelRGB>& dst);


#endif
