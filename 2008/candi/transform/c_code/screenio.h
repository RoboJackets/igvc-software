#ifndef _SCREENIO_H_
#define _SCREENIO_H_
#include "image.h"
extern Image* screen0;

#if __cplusplus
	#include "Buffer2D.h"
	#include "PixelRGB.h"
	extern Buffer2D<PixelRGB>* screen;
	extern "C" {
#endif
	void screeninit (void);
#if __cplusplus
}
#endif

#endif
