#include "screenio.h"
#include "imageutils.h"
#include "texcode.h"
#include "getwhim.h"
#include "getorim.h"
#include "curtain.h"
#include "blackglobals.h"
#include <stdio.h>
#include <stdlib.h>
extern "C" {
#include "utilfuncs.h"
}


static Image * infr1=new Image;
static Buffer2D<PixelRGB> whim,orim,fr1,barim;
//static Buffer2D<PixelRGB> * fr1;
//static Buffer2D<PixelRGB> * barim;


void blackmain() {
	
	/* get image */
	//fr1=new Buffer2D<PixelRGB>;
	long long t1=currentTimeMicros();
	ImageLoad ("12.bmp", infr1);
	fr1.copyFrom (infr1->width,infr1->height, (PixelRGB*) infr1->data);
	free (infr1->data);					//don't leak memory!
	fr1.shrink(5);
	/* process */
	orim = getorim (fr1);
	whim = getwhim (fr1);
	//Buffer2D<bool> test=orim.toBool();
	//Buffer2D<PixelRGB> test2=test.toRGB();
	
	blackout(curtain (whim, orim),fr1);
	//barim = fr1;

    barim.copyFrom(fr1);
    barim.grow(5);
	/* put image into screen */
	screen = &barim;
	/* put screen into graphics card*/
	
	double diff=currentTimeMicros()-t1;
	printf("framerate: %.2f\n",1.0/diff*1000000.0);
	
	NextFrame();
	//delete fr1;
}

