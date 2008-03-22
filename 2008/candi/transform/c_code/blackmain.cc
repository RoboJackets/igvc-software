#include "screenio.h"
#include "imageutils.h"
#include "texcode.h"
#include "getwhim.h"
#include <stdio.h>
extern "C"{
#include "utilfuncs.h"
}

static Image * infr1=new Image;
static Buffer2D<PixelRGB> fr1;
static b2drgb whim;


extern "C" void blackmain(){

	ImageLoad("1.bmp", infr1);

	fr1.copyFrom(infr1->width,infr1->height,(PixelRGB*)infr1->data);

	getwhim(fr1, whim);

	screen=&whim;

	NextFrame();
}
