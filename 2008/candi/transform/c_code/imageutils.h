#ifndef _IMAGEUTILS_H_
#define _IMAGEUTILS_H_


#include "Buffer2D.h"
#include "PixelRGB.h"
#include "image.h"

Buffer2D<PixelRGB>* imageAsBuffer2D(Image* img);
Buffer2D<PixelRGB> imageAsStaticBuffer2D(Image* img);

#endif
