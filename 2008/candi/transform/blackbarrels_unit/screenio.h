#ifndef _SCREENIO_H_
#define _SCREENIO_H_
#include "image.h"

namespace blackbarrels{

extern Image* screen0;
}
#if __cplusplus
	#include "Buffer2D.h"
	#include "PixelRGB.h"
	namespace blackbarrels{
	extern Buffer2D<PixelRGB>* screen;
	}
	extern "C" {
#endif
	void screeninit (void);
#if __cplusplus
}
#endif


#endif
