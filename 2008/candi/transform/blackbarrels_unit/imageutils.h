#ifndef _IMAGEUTILS_H_
#define _IMAGEUTILS_H_


#include "Buffer2D.h"
#include "PixelRGB.h"
#include "image.h"
namespace blackbarrels{

Buffer2D<PixelRGB>* imageAsBuffer2D (Image* img);
Buffer2D<PixelRGB> imageAsStaticBuffer2D (Image* img);
void RGBtoBool (Buffer2D<PixelRGB>& img, Buffer2D<bool>& dst);
void booltoRGB (Buffer2D<bool>& img, Buffer2D<PixelRGB>& dst);
void booland(Buffer2D<bool>& img1,Buffer2D<bool>& img2);
void blackout (Buffer2D<bool>& mask, Buffer2D<PixelRGB>& img);
void orangeout (Buffer2D<bool>& mask, Buffer2D<PixelRGB>& img);
}
#endif
