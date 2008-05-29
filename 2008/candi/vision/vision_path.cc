
#include "vision_path.h"
#include "vision_color.h"
#include "vision_line_blobber.h"
#include "vision.h"
#include "vision_util.h"	
#include <math.h>		// for sqrt
#include <stdlib.h>		// for abs

#define ROBOT_WIDTH 24 //pixels wide


void visGenPath();
Buffer2D<bool> visPathView;
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

	
	if (goodFirst){		//starting pixel is good
		good=1;
		for(;x>=end;x--){	//scan left and check
			if(good){
				if(checkPaulBlobPixel(x,y)){
					//set good
					visPathView.set(x,y,1);
				}
				else{
					//set bad
					good=0;
					visPathView.set(x,y,0);
				}
			}
			else{		//all the rest are bad
				//set bad 
				visPathView.set(x,y,0);
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
					visPathView.set(x,y,1);
				}
				else{
					//set bad
					visPathView.set(x,y,0);
				}
			}
			else if (good==1){	//a good spot appeared, check for bad again
				if(checkPaulBlobPixel(x,y)){
					//set good
					visPathView.set(x,y,1);
					}
				else{
					//set bad
					good=0;
					visPathView.set(x,y,0);
				} 
			}
			else{ //all the rest are bad
				//set bad
				visPathView.set(x,y,0);
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
					visPathView.set(x,y,1);
				}
				else{
					//set bad
					good=0;
					visPathView.set(x,y,1);
				}
			}
			else{		//all the rest are bad
				//set bad 
				visPathView.set(x,y,0);
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
					visPathView.set(x,y,1);
				}
				else{
					//set bad
					visPathView.set(x,y,0);
				}
			}
			else if (good==1){	//a good spot appeared, check for bad again
				if(checkPaulBlobPixel(x,y)){
					//set good
					visPathView.set(x,y,1);
					}
				else{
					//set bad
					good=0;
					visPathView.set(x,y,0);
				} 
			}
			else{ //all the rest are bad
				//set bad
				visPathView.set(x,y,0);
			}
		}		
	}
}



Point2D<int> robotWidthScan(){
	Point2D<int> goal;
	int width = visRaw.width;
	int height = visRaw.height;
	int center = width/2;
	int startx = center - ROBOT_WIDTH/2;
	int endx = startx + ROBOT_WIDTH;
	int y = height-1;
	int x = startx;
	int half = ROBOT_WIDTH/2;
	
	goal.x=-1;
	goal.y=-1;
	
	/* 	scan from bottom center of image upward,
		checking to see if the width of the robod can progress 
		any further up the image, sliding left/right as needed
	*/
	for( ; y >=0 ; y--){
		
		//check left,move right
		for(x=startx; x<width-ROBOT_WIDTH; x++){
			if(!visPathView.get(x,y)){
				x++;	//slide right
			}
			else{
				goal.x = x+half;
				goal.y = y;
				break;				
			}
		}
		if(visPathView.get(x+ROBOT_WIDTH,y)&&visPathView.get(x+half,y)){
			startx=x;
			endx=startx+ROBOT_WIDTH;
			paulBlob.set(x,y,Pixel(200,0,0));
			continue;	//stop if we fit
		}
			
		//check right,move left
		for(x=endx-1; x>=0; x--){
			if(!visPathView.get(x,y)){
				x--;	//slide left
			}
			else{
				goal.x = x-half;
				goal.y = y;
				break;				
			}
		}		
		if(visPathView.get(x-ROBOT_WIDTH,y)&&visPathView.get(x-half,y)){
			startx=x-ROBOT_WIDTH;
			endx=x;
			paulBlob.set(x,y,Pixel(0,0,200));
			continue;	//stop if we fit		
		}
		else
			break;		//we dont fit left or right

		
	}
	
	//sanity check goal
	if(goal.x==-1||goal.y==-1){
		//not good
	}
	else{
		//found goal
		if(!visPathView.get(goal.x,goal.y)){
			goal.x=goal.y=-1;	//not good
		}
		else{
		
			//debug/////////
			Graphics g(&paulBlob);
			g.setColor(Pixel(20, 20, 20));	//dark
			g.drawLine(center,height-1,goal.x,goal.y);
			g.drawLine(goal.x-half,goal.y,goal.x+half,goal.y);
			////////////////
		
			//good
			//flip y coordinate
			goal.y=height-goal.y;
		}

	}

	// return center of the scan's final location
	// return -1 on error
	return goal;
}




