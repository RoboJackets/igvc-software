#include "curtain.h"
#include "blackglobals.h"
#include "Point2D.h"

static Buffer2D<PixelRGB> barim;
static Buffer2D<bool> boolimg;
static Buffer2D<bool> boolarr;

b2dbool& RGBtoBool (Buffer2D<PixelRGB> img) {
	int ii;
	int buffLength = img->width * img->height;

	/*standard boolean conversion*/
	for(ii = 0; ii < buffLength; ii++){
		if(img[ii].r > 0 || img[ii].g > 0 || img[ii].b > 0)
			boolim[ii] == 1;
		else
			boolim[ii] == 0;
	}
	
	return boolim;
}

b2dbool& dialate1D (Buffer2D<bool> arr) {
	int ii;
	int buffLength = arr->width * arr->height;

	/*if current pos or neighbor pos is 1, then current pos is 1*/
	for(ii = 1; ii < buffLength-1; ii++){
		if(arr[ii-1] || arr[ii] || arr[ii+1){
			boolarr[ii] = 1;
		}
		else
			boolarr[ii] = 0;
	}	
	
	return boolarr;
}

b2drgb& curtain (Buffer2D<PixelRGB> whim, Buffer2D<PixelRGB> orim) {
	
	int ii = 0;
	int orangeIndex = 0;
	int buffLength = orim->width * orim->height;
	bool isOrange = true;
	
	/*get boolean orange image*/
	boolim = RGBtoBool(orim);
	
	/*loop untill no more orange pixels*/
	while(isOrange){
	
		/*locate first index of first orange pixel*/
		orangeIndex = -1;
		for(ii = 0; ii < buffLength; ii++){
			if(boolim){
				orangeIndex = ii;
				break;
			}
		}//end first orange index
		
		/*quit if there is no more orange*/
		if(orangeIndex == -1){
			isOrange == false;
			break;
		}
		
		
	

		
		
	}//end while



	return barim;
}

Buffer2D<bool>* cutout(int x,int y,Buffer2D<bool>& img){


}

