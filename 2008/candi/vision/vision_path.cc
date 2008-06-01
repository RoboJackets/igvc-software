
#include "vision_path.h"
#include "vision_color.h"
#include "vision_line_blobber.h"
#include "vision.h"
#include "vision_util.h"	
#include "vision_navigation.h"

#include <math.h>		// for sqrt
#include <stdlib.h>		// for abs

#define ROBOT_WIDTH 24 //pixels wide


void visGenPath();
Buffer2D<bool> visPathView;
int checkPaulBlobPixel(int x, int y);
void scanFillLeft(int middleX, int y, int goodFirst, int end, int blackout);
void scanFillRight(int middleX, int y, int goodFirst, int end, int blackout);





void visGenPath(void){
	int width = visRaw.width;
	int height = visRaw.height;
	int goodFirst = 1;
	int x = width/2;
	visPathView.resize(width, height);

	int blackout = 0;
	
	// scan bottom to top in rows; white = path; black = bad	
	for(int y = height-3; y >0 ; y--){
		if(checkPaulBlobPixel(x,y)){	//check starting point in middle
			goodFirst=1;
		}
		else{
			goodFirst=0;
			blackout=1;
		}
		

		//scan left then right & generate visPathView image
		scanFillLeft (x,y,goodFirst,0      ,blackout);
		scanFillRight(x,y,goodFirst,width-1,blackout);		

			
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
	//else if (p.red==255 && p.green==0 && p.blue==255){
	//	good = 0;	
	//}	
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
void scanFillLeft(int middleX, int y, int goodFirst, int end, int blackout){
	int x=middleX;	
	int good;

	if (blackout){		
		for(;x>=end;x--){	//fill black
			//set bad 
			visPathView.set(x,y,0);
		}
		return;
	}
	
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
		for(;x>=end;x--){	//scan left and check
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
void scanFillRight(int middleX, int y, int goodFirst, int end, int blackout){
	int x=middleX;	
	int good;

	if (blackout){		
		for(;x<end;x++){	//fill black
			//set bad 
			visPathView.set(x,y,0);
		}
		return;
	}
	
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
		for(;x<end;x++){	//scan right and check
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
	int y = height-3;
	int x = startx;
	int half = ROBOT_WIDTH/2;
	int i;
	
	
	// return -1 on failure
	goal.x=-1;
	goal.y=-1;

	
	/* 	scan from bottom center of image upward,
		checking to see if the width of the robot can progress 
		any further up the image, sliding left/right as needed
	*/
	for( ; y >0 ; y-- ){
		
		//check left, move right
		for(x=startx; x<width-ROBOT_WIDTH-1; x++){
			if(!visPathView.get(x,y) || !visPathView.get(x+half,y) ){
				x++;	//slide right
				startx=x;
				endx=x+ROBOT_WIDTH;
			}
			else{
				startx=x;
				endx=x+ROBOT_WIDTH;
				break;				
			}
		}

		for( i = startx; i<endx; i++){ //scan along width to not cross over boundary
			if(!visPathView.get(i,y)){
				break;
			}
			else{
				goal.x = i-half;
				goal.y = y;
				//paulBlob.set(i,y,Pixel(0,0,255));
			}
		}
		if(i>=endx){
			//success, keep going up
			continue;
		}
				
		//check right, move left
		for(x=endx; x>0+ROBOT_WIDTH; x--){
			if(!visPathView.get(x,y)  || !visPathView.get(x-half,y) ){
				x--;	//slide left
				startx=x-ROBOT_WIDTH;
				endx=x;
			}
			else{
				startx=x-ROBOT_WIDTH;
				endx=x;
				break;				
			}
		}	
		for( i = endx; i>startx; i--){ //scan along width to not cross over boundary
			if(!visPathView.get(i,y)){
				break;
			}
			else{
				goal.x = i+half;
				goal.y = y;
				//paulBlob.set(i,y,Pixel(0,0,185));
			}
		}
		if(i<=startx){
			//success, keep going up
			continue;
		}	
		else{
			break;		//we dont fit left or right
		}
	}
	
	//sanity check goal
	if(goal.x==-1||goal.y==-1){
		//not good
	}
	else{
		//found goal
		if(!visPathView.get(goal.x,goal.y)){
			goal.x=goal.y=-1;	//not good, error in scanning
		}
		else{
		
			//=== debug: show goal ===//
			//Graphics g(&paulBlob);
			Graphics g(&visNavigationParams);
			g.setColor(Pixel(20, 20, 20));	//dark
			g.drawLine(center,height-1,goal.x,goal.y);
			g.drawLine(goal.x-half,goal.y,goal.x+half,goal.y);
			//========================//
		
			//good
			//flip y coordinate for path planning
			//goal.y=height-goal.y;
		}
	}

	// return center of the scan's final location
	// return -1 on error
	return goal;
}


void visPathControlMotors(Point2D<int> goal){


	// Initialize the navigation-param view with the raw camera image
	// (so that we can annotate on top of it)
	Graphics g(&visNavigationParams);
	//Graphics g(&paulBlob);
	//visNavigationParams.copyFrom(visRaw); //copyFrom now called in vision.cc
	//visNavigationParams.copyFrom(paulBlob); //
	
	// Annotate the colors of the pixels in the raw camera image
	// (since the danger map that navigation relys on is generated
	//  from the set of identified pixel colors)
	{
		// Annotate standard colors
		visAnnotatePixelColors(visNavigationParams);
	}
	
	

	
	// find angle from center of image
	//printf("goal(%d,%d) with (0,0) upper left\n",goal.x,visRaw.height-goal.y);
	int halfWidth = (visRaw.width/2);
	int bestPath_id = (visRaw.width - goal.x);


	/*
	 * Direct the motors (while in autonomous mode)
	 * to drive in the direction of the best path
	 * with a thrust inversely proportional to the
	 * sharpness of the turn required to follow
	 * the best path
	 *
	 * NOTE: The interpretation of the constructed
	 * DriveMotion value is affected by the algorithm
	 * used to convert it into the final MotorOutput.
	 */
	DriveMotion nav_driveMotion;
	MotorOutput nav_motorOutput;
	{
		
		// Swivel is proportional to the angle of the best path.
		// WARNING: Assumes that the navigation view cone is symmetric about the vertical axis.
		int nav_swivel = 127*(halfWidth-bestPath_id)/halfWidth; //= 127 * (NAV_PATH__CENTER_PATH_ID - bestPath_id)/NAV_PATH__CENTER_PATH_ID;
		if (nav_swivel < -128)
			nav_swivel = -128;
		if (nav_swivel > 127)
			nav_swivel = 127;

		//printf("goal id %d	nav swivel %d\n",bestPath_id,nav_swivel);		
		
		int nav_thrust = 177;// = 127;
		
		nav_driveMotion = DriveMotion(nav_thrust, nav_swivel);
		nav_motorOutput = nav_driveMotion.toMotorOutput();
		
		autonomousModeMotorOutput = nav_motorOutput;
	}
	
	/*
	 * Display the output DriveMotion and MotorOutput
	 */
	{
		const int DRIVE_BAR_THICKNESS = 5;
		
		//int halfWidth = visRaw.width/2;
		int halfHeight = visRaw.height/2;
		
		int thrustExtent = nav_driveMotion.thrust * halfHeight/128;
		int swivelExtent = nav_driveMotion.swivel * halfWidth/128;
		
		int leftSpeedExtent = nav_motorOutput.leftSpeed * halfHeight/128;
		int rightSpeedExtent = nav_motorOutput.rightSpeed * halfHeight/128;
		
		/* Draw the swivel bar along the top and bottom of the view */
		g.setColor(Pixel(0, 0, 200));	// dark blue
		g.fillRect_rational(
			(swivelExtent > 0) ? halfWidth : (halfWidth + swivelExtent),
			0,
			abs(swivelExtent),
			DRIVE_BAR_THICKNESS);
		g.fillRect_rational(
			(swivelExtent > 0) ? halfWidth : (halfWidth + swivelExtent),
			visRaw.height - DRIVE_BAR_THICKNESS,
			abs(swivelExtent),
			DRIVE_BAR_THICKNESS);
		
		/* Draw the thrust bar along the left and right of the view */
		g.setColor(Pixel(200, 0, 0));	// dark red
		g.fillRect_rational(
			0,
			(thrustExtent < 0) ? halfHeight : (halfHeight - thrustExtent),
			DRIVE_BAR_THICKNESS,
			abs(thrustExtent));
		g.fillRect_rational(
			visRaw.width - DRIVE_BAR_THICKNESS,
			(thrustExtent < 0) ? halfHeight : (halfHeight - thrustExtent),
			DRIVE_BAR_THICKNESS,
			abs(thrustExtent));
		
		/* Draw the left/right wheel speed bars along the left and right of the view */
		g.setColor(Pixel(225, 225, 225));	// light grey
		g.fillRect_rational(
			DRIVE_BAR_THICKNESS,
			(leftSpeedExtent < 0) ? halfHeight : (halfHeight - leftSpeedExtent),
			DRIVE_BAR_THICKNESS,
			abs(leftSpeedExtent));
		g.fillRect_rational(
			visRaw.width - (DRIVE_BAR_THICKNESS * 2),
			(rightSpeedExtent < 0) ? halfHeight : (halfHeight - rightSpeedExtent),
			DRIVE_BAR_THICKNESS,
			abs(rightSpeedExtent));
	}

}
	




