#include "curtain.h"
#include "blackglobals.h"
#include "Point2D.h"
#include <string.h>
#include <stdlib.h>
#include "screenio.h" //for debugging
#include "texcode.h"

static Buffer2D<bool> barim;
static Buffer2D<bool> orim;
static Buffer2D<bool> whim;
static Buffer2D<PixelRGB> barimout;
Buffer2D<bool>& clear(Buffer2D<bool>& im);
void dropperFlopper (Buffer2D<bool>& cr,Buffer2D<bool>& orim,Buffer2D<bool>& whim, Buffer2D<bool>& dstim,int line);

void booltoRGB (Buffer2D<bool>& img, Buffer2D<PixelRGB>& dst){
	int ii;
	int buffLength = dst.width * dst.height;

	/*standard reverse boolean conversion*/
	for(ii = 0; ii < buffLength; ii++){
		if(img[ii])
			dst[ii].r = dst[ii].g = dst[ii].b = 255;
		else
			dst[ii].r = dst[ii].g = dst[ii].b = 0;
	}	
}

void RGBtoBool (Buffer2D<PixelRGB>& img, Buffer2D<bool>& dst) {
	int ii;
	int buffLength = img.width * img.height;

	/*standard boolean conversion*/
	for(ii = 0; ii < buffLength; ii++){
		if(img[ii].r > 0 || img[ii].g > 0 || img[ii].b > 0)
			dst[ii] = 1;
		else
			dst[ii] = 0;
	}
}


//dialates the line IN PLACE
void dilate1D (Buffer2D<bool>& arr) {
	int ii;
	int buffLength = arr.width * arr.height;
	static Buffer2D<bool> boolarr;
	boolarr.resizeToMatch(arr);
	/*if current pos or neighbor pos is 1, then current pos is 1*/
	for(ii = 1; ii < buffLength-1; ii++){
		if(arr[ii-1] || arr[ii] || arr[ii+1]){
			boolarr[ii] = 1;
		}
		else
			boolarr[ii] = 0;
	}	
	boolarr[0]=boolarr[buffLength-1]=0;
	memcpy(arr.data, boolarr.data, sizeof(bool)*buffLength);
}

b2drgb curtain (Buffer2D<PixelRGB>& whimin, Buffer2D<PixelRGB>& orimin) {
	
	int ii = 0;
	int orangeIndex = 0;
	int buffLength = orimin.width * orimin.height;
	bool isOrange = true;
	orim.resize(orimin.width,orimin.height);
	whim.resizeToMatch (orim);
	
	/*get boolean images*/
	RGBtoBool(orimin, orim);
	RGBtoBool(whimin, whim);
	barim.copyFrom(orim);
	barimout.resize(orim.width,orim.height);
	
	/*loop untill no more orange pixels*/
	ii = 0;
	while(isOrange){
	
		/*locate first index of first orange pixel*/
		orangeIndex = -1;
		for(; ii < buffLength; ii++){
			if(orim[ii]){
				orangeIndex = ii;
				break;
			}
		}//end first orange index
		
		/*quit if there is no more orange*/
		if(orangeIndex == -1){
			isOrange = false;
			break;
		}
		
		//cut the found region out to cr from orim
		Buffer2D<bool> cr=cutout(orangeIndex,orim);
		/*uncomment to debug first cr
		booltoRGB(cr,barimout);
		break;*/
		for(int y=0; y < cr.height-1; y++){
			Buffer2D<bool>* thisln = cr.getLine(y);
			dilate1D(*thisln);
			delete thisln;
			dropperFlopper(cr,orim,whim,barim,y);
		}
	

		
		
	}//end while
	
	booltoRGB(barim,barimout);
	return barimout;
}

Buffer2D<bool>& cutout(int idx,Buffer2D<bool>& img) {
	int x=idx%img.width;
	int y=idx/img.width;
	static Point2D<int>* plst=0;		// pixel list
	static int plsz=0;					// pixel list size
	static Buffer2D<bool>  cr;			//current region of interest
	Point2D<int> xmax(x, y);
	Point2D<int> xmin(x, y);
	Point2D<int> ymax(x, y);
	Point2D<int> ymin(x, y);
	const int width = img.width;
	const int height = img.height;
	const int numElem = img.numElements();
	
	if (numElem != cr.numElements()) {
		plst = new Point2D<int>[numElem];
		plsz = numElem;
		cr.resizeToMatch (img) ;
	}
	
	//fill cr with 0's
	clear(cr);
		
	// lets cut!
	int cp=0;		//current point index
	int np=1;		//num points so far found, starts with 1
	//read in initial point
	
	
	plst[cp].x=x;	
	plst[cp].y=y;
	
	img.data[plst[cp].x+plst[cp].y*width]=0;		//clear initial point
	for(;cp<np;cp++){						//run till out of points
		//clear and record 4 neighbors
		{
			//cache base x and y (hopefully in registers)			 
				int cx=plst[cp].x;
				int cy=plst[cp].y;
			
			//white top
			{
				if(img.data[cx+(cy+1)*width]){
					img.data[cx+(cy+1)*width]=0;	//clear it
					plst[np].y=cy+1;			//add to found list
					plst[np].x=cx;
					np++;
				}
			}
			//white right
			{
				if(img.data[cx+1+(cy)*width]){		
					img.data[cx+1+(cy)*width]=0;	//clear it
					plst[np].y=cy;			//add to found list
					plst[np].x=cx+1;
					np++;
				}
			}
			//white bottom
			{
				if(img.data[cx+(cy-1)*width]){
					img.data[cx+(cy-1)*width]=0;	//clear it
					plst[np].y=cy-1;			//add to found list
					plst[np].x=cx;
					np++;
				}
			}
			//white left
			{
				if(img.data[cx-1+(cy)*width]){		
					img.data[cx-1+(cy)*width]=0;	//clear it
					plst[np].y=cy;			//add to found list
					plst[np].x=cx-1;
					np++;
				}
			}
		}
	}
	//pump out region to cr
	for(cp=0;cp<np;cp++){
		cr.data[plst[cp].x+plst[cp].y*width]=1;
	}
	//printf("points : %d\n",np);
	return cr;
}


void dropperFlopper (Buffer2D<bool>& cr,Buffer2D<bool>& orim,Buffer2D<bool>& whim, Buffer2D<bool>& dstim,int line) {
	
	Buffer2D<bool>& curln 	= *cr.	getLine(line	);
	Buffer2D<bool>& nxtcrln = *cr.	getLine(line+1	);
	Buffer2D<bool>& nxtwhln = *whim.getLine(line+1	);
	Buffer2D<bool>& dstln 	= *dstim.getLine(line+1	);
	Buffer2D<bool>& orln 	= *orim.getLine(line+1	);
	int buffLength = curln.width * curln.height;
	int ii;
	
	/*and the current line with the next one for dropping in*/
	for(ii = 0; ii < buffLength; ii++){
		if(curln[ii]){
			if(nxtwhln[ii]){
				nxtcrln[ii] = 1 ;
				dstln[ii] = 1 ;
				
			}
		}
	}	 
	delete &curln;
	delete &nxtcrln;
	delete &nxtwhln;
	delete &dstln;
	delete &orln;
}

Buffer2D<bool>& clear(Buffer2D<bool>& im){
	// Free old data buffer (if one was allocated)
	if (im.data != NULL) {
		delete[] im.data;
	}

	// Allocate new data buffer (if width & height are non-zero)
	if ( (im.width != 0) && (im.height != 0)) {
		im.data = (bool*)calloc(im.width * im.height,sizeof(bool));
	} else {
		im.data = NULL;
	}
}




