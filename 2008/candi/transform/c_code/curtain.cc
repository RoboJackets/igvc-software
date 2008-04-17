#include "curtain.h"
#include "blackglobals.h"
#include "Point2D.h"
#include <string.h>
#include <stdlib.h>
#include "screenio.h" //for debugging
#include "texcode.h"
#include "imageutils.h"

static Buffer2D<bool> barim;
static Buffer2D<bool> orim;
static Buffer2D<bool> whim;
static Buffer2D<PixelRGB> barimout;
Buffer2D<bool>& clear(Buffer2D<bool>& im);
void dropperFlopper (Buffer2D<bool>& cr,Buffer2D<bool>& orim,Buffer2D<bool>& whim, Buffer2D<bool>& dstim,int line);

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
static int debugnum=1;
static int debugrun=1;
Buffer2D<bool>& curtain (Buffer2D<PixelRGB>& whimin, Buffer2D<PixelRGB>& orimin) {
	
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
	//barimout.resize(orim.width,orim.height);
	
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
			debugnum=1;
			debugrun=1;
			break;
		}
		
		//cut the found region out to cr from orim
		Buffer2D<bool> cr=cutout(orangeIndex,orim);
		
		for(int y=0; y < cr.height-1; y++){
			Buffer2D<bool>* thisln = cr.getLine(y);
			dilate1D(*thisln);
			delete thisln;
			dropperFlopper(cr,orim,whim,barim,y);
		}
		
		
		//demomode
		debugrun--;
		if(debugrun==0){
			debugnum++;
			debugrun=debugnum;
			break;
		}
		
		
	}//end while
	
	//booltoRGB(barim,barimout);
	return barim;
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
	
	//put black borders on
	for(int i=0;i<height;i++){
		img[i*width]=0;
		img[(i+1)*width-1]=0;
	}
	memset((void*)&(img[0]),0,width*sizeof(bool));
	memset((void*)&(img[(height-1)*width]),0,width*sizeof(bool));
	
	img.data[plst[cp].x+plst[cp].y*width]=0;		//clear initial point
	for(;cp<np;cp++){						//run till out of points
		//clear and record 4 neighbors
		{
			//cache base x and y (hopefully in registers)			 
				int cx=plst[cp].x;
				int cy=plst[cp].y;
				
			//variable for index
				int index = 0;
			
			//white top
			{
				index = cx+(cy+1)*width;
				if(img.data[index]){
					img.data[index]=0;	//clear it
					plst[np].y=cy+1;			//add to found list
					plst[np].x=cx;
					np++;
				}
			}
			//white right
			{
				index = cx+1+(cy)*width;
				if(img.data[index]){		
					img.data[index]=0;	//clear it
					plst[np].y=cy;			//add to found list
					plst[np].x=cx+1;
					np++;
				}
			}
			//white bottom
			{
				index = cx+(cy-1)*width;
				if(img.data[index]){
					img.data[index]=0;	//clear it
					plst[np].y=cy-1;			//add to found list
					plst[np].x=cx;
					np++;
				}
			}
			//white left
			{
				index = cx-1+(cy)*width;
				if(img.data[index]){		
					img.data[index]=0;	//clear it
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


void dropperFlopper (Buffer2D<bool>& cr,Buffer2D<bool>& orim,Buffer2D<bool>& whim, Buffer2D<bool>& dstim,int y) {
	int buffLength = cr.width;
	int w=buffLength;
	int ii,yc,ync;
	yc=w*y;
	ync=w*(y+1);
	
	/*and the current line with the next one for dropping in*/
	for(ii = 0; ii < buffLength; ii++){
		if(cr[yc+ii]){
			if(whim[ync+ii]){
				cr[ync+ii] = 1 ;
				dstim[ync+ii] = 1 ;
				
			}
		}
	}	 
}

Buffer2D<bool>& clear(Buffer2D<bool>& im){	
	if (im.data != NULL) {
		memset(im.data, 0, sizeof(bool)*(im.width * im.height));
	}
}




