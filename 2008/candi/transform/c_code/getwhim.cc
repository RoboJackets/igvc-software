#include "stdio.h"
#include "image.h"
#include "Buffer2D.h"
#include "PixelRGB.h"
#include "getwhim.h"

//typedef  Buffer2D<PixelRGB> b2drgb;
static Buffer2D<PixelRGB> whim;


int numruns=0;
b2drgb& getwhim(b2drgb& im){

	int r,g,b,orn;
	char R,G,B;
	whim.copyFrom(im);
	for(int i=0;i<(im.width)*(im.height);i++){
		r=whim[i].r;
		g=whim[i].g;
		b=whim[i].b;
		orn=r-b;
		
		// Clear out image
		whim[i].r=0;
		whim[i].g=0;
		whim[i].b=0;
		R=G=B=0;
		
		if(g!=0 && 256*b/g>171 && b>102 && 256*g/(r+b)<154){
			R=255;
		}
			
		whim[i].r=R;
		whim[i].g=G;
		whim[i].b=B;
				
	}
	
	numruns++;
	
	printf("frame: %d\n",numruns);
	
	
	
	return whim;


}
