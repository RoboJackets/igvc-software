
#include "vision_path.h"
#include "vision_color.h"
#include "vision_line_blobber.h"
#include "vision.h"
#include "vision_util.h"	
#include <math.h>		// for sqrt
#include <stdlib.h>		// for abs

void visGenPath();
Buffer2D<Pixel> visPathView;
int checkPaulBlobPixel(int x, int y);
void scanFillLeft(int middleX, int y, int goodFirst, int end);
void scanFillRight(int middleX, int y, int goodFirst, int end);

void visGenPath(void){
	int width = visRaw.width;
	int height = visRaw.height;
	int goodFirst = 1;
	int x = width/2;
	visPathView.resize(width, height);

	
	// scan bottom to top in rows; white = path; black = bad	
	for(int y = height-1; y >=0 ; y--){
		if(checkPaulBlobPixel(x,y)){	//check starting point in middle
			goodFirst=1;
		}
		else{
			goodFirst=0;
		}
		
		//scan left then right & generate visPathView image
		scanFillLeft(x,y,goodFirst,0);
		scanFillRight(x,y,goodFirst,width);		
			
	}//y

	
	
}//visGenPath


int checkPaulBlobPixel(int x, int y){
	int good;
	Pixel p = paulBlob.get(x,y);

	// red: from vision line blobber
	//if (p.red==200 && p.green==0 && p.blue==0){
	//	good = 0;	
	//}
	// white: from vision line blobber
	if (p.red==255 && p.green==255 && p.blue==255){
		good = 0;	
	}	
	// purple: from shader
	else if (p.red==255 && p.green==0 && p.blue==255){
		good = 0;	
	}	
	// orange from shader
	else if (p.red==255 && p.green==128 && p.blue==0){
		good = 0;	
	}
	// black from transform
	else if (p.red==0 && p.green==0 && p.blue==0){
		good = 0;					
	}
	// probably good
	else{
		good = 1;
	}
	
	return good;
}

/*this function scans from the center of the paulBlob image to end
	checking for good / bad pixels, setting the visPathView image accordingly */
void scanFillLeft(int middleX, int y, int goodFirst, int end){
	int x=middleX;	
	int good;
	Pixel p2;
	
	if (goodFirst){		//starting pixel is good
		good=1;
		for(;x>=end;x--){	//scan left and check
			if(good){
				if(checkPaulBlobPixel(x,y)){
					//set good
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
				}
				else{
					//set bad
					good=0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				}
			}
			else{		//all the rest are bad
				//set bad 
				p2.red=p2.green=p2.blue=0;
				visPathView.set(x,y,p2);
			}
		}
	}
	else {		//starting pixel is bad
		good=2;
		for(;x>=0;x--){	//scan left and check
			if(good==2){	//in bad spot, check for good spot
				if(checkPaulBlobPixel(x,y)){
					//set good
					good=1;
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
				}
				else{
					//set bad
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				}
			}
			else if (good==1){	//a good spot appeared, check for bad again
				if(checkPaulBlobPixel(x,y)){
					//set good
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
					}
				else{
					//set bad
					good=0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				} 
			}
			else{ //all the rest are bad
				//set bad
				p2.red=p2.green=p2.blue=0;
				visPathView.set(x,y,p2);
			}
		}		
	}
}
/*this function scans from the center of the paulBlob image to end
	checking for good / bad pixels, setting the visPathView image accordingly */
void scanFillRight(int middleX, int y, int goodFirst, int end){
	int x=middleX;	
	int good;
	Pixel p2;
	
	if (goodFirst){		//starting pixel is good
		good=1;
		for(;x<end;x++){	//scan right and check
			if(good){
				if(checkPaulBlobPixel(x,y)){
					//set good
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
				}
				else{
					//set bad
					good=0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				}
			}
			else{		//all the rest are bad
				//set bad 
				p2.red=p2.green=p2.blue=0;
				visPathView.set(x,y,p2);
			}
		}
	}
	else {		//starting pixel is bad
		good=2;
		for(;x>=0;x--){	//scan right and check
			if(good==2){	//in bad spot, check for good spot
				if(checkPaulBlobPixel(x,y)){
					//set good
					good=1;
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
				}
				else{
					//set bad
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				}
			}
			else if (good==1){	//a good spot appeared, check for bad again
				if(checkPaulBlobPixel(x,y)){
					//set good
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
					}
				else{
					//set bad
					good=0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				} 
			}
			else{ //all the rest are bad
				//set bad
				p2.red=p2.green=p2.blue=0;
				visPathView.set(x,y,p2);
			}
		}		
	}
}


