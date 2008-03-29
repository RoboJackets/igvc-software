#include "curtain.h"

static Buffer2D<PixelRGB> barim;

b2drgb& curtain (Buffer2D<PixelRGB> whim, Buffer2D<PixelRGB> orim) {
	
	int ii = 0;
	int orangeIndex = 0;
	int buffLength = orim->width * orim->height;
	bool isOrange = true;
	
	/*loop untill no more orange pixels*/
	while(isOrange){
	
		/*locate first index of first orange pixel*/
		orangeIndex = -1;
		for(ii = 0; ii < buffLength; ii++){
			if(orim[ii].r == 255){
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
