#ifndef getwhim_h
#define getwhim_h
#include "Buffer2D.h"
#include "PixelRGB.h"

namespace blackbarrels{


typedef  Buffer2D<PixelRGB> b2drgb;

b2drgb& getwhim (b2drgb& im);

}

#endif
