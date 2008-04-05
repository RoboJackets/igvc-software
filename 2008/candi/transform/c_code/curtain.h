#ifndef curtain_h
#define curtain_h


//#include "Pixel.h"
#include "stdio.h"
#include "image.h"
#include "Buffer2D.h"
#include "PixelRGB.h"

typedef  Buffer2D<PixelRGB> b2drgb;
typedef  Buffer2D<bool> b2dbool;

b2drgb& curtain (Buffer2D<PixelRGB> whim, Buffer2D<PixelRGB> orim);



#endif
