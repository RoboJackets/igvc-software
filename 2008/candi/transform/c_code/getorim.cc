#include "stdio.h"
#include "image.h"
#include "Buffer2D.h"
#include "PixelRGB.h"
#include "getorim.h"


static Buffer2D<PixelRGB> orim;


static int numruns=0;
b2drgb& getorim (b2drgb& im) {

	int r,g,b,orn;
	char R,G,B;
	orim.copyFrom (im);
	for (int i=0;i< (im.width) * (im.height);i++) {
		
		//get pixel data
		r=orim[i].r;
		g=orim[i].g;
		b=orim[i].b;
		orn=r-b;

		// Clear out image
		orim[i].r=0;
		orim[i].g=0;
		orim[i].b=0;
		R=G=B=0;

		//thresholding
		if (g<r-20) {
			R=255;
			G=255;
			B=255;
		}

		//update image
		orim[i].r=R;
		orim[i].g=G;
		orim[i].b=B;
	}

	numruns++;

	//printf ("frame: %d\n",numruns);

	return orim;
}
