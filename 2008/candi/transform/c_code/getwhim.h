#ifndef getwhim_h
#define getwhim_h
#include "Buffer2D.h"
#include "PixelRGB.h"


/* --- Public C Functions --- */

typedef  Buffer2D<PixelRGB> b2drgb;

void getwhim(b2drgb& im, b2drgb& whim);



#endif
