#include "vision.h"
#include "vision_color.h"
#include "vision_barrels.h"
#include "vision_navigation.h"
#include "vision/vision_line_blobber.h"
#include "vision/vision_path.h"
#include "pathplan/pathplan.h"

#include "Camera.h"
#include "Pixel.h"
#include "Buffer2D.h"
#include <math.h>

#if USE_VISION_MUTEX
QMutex visMutex;
#endif

// ### BUFFERS ###

// Primary image buffer
Buffer2D<Pixel> visRaw;

// Auxilary testing view (for debugging purposes)
Buffer2D<Pixel> visTestViewContent;

// ### MAIN VISION PROCESSING ###

#include "vision_util.h"

#define closenessThresh 80 //pixels from bottom
Point2D<int> goal;

// Performs all vision processing.
// 
// At present it appears that this processing does NOT
// affect the course that the autonomous mode drives in.
void visFrame()
{
	// Lock the vision mutex before doing anything else.
	//
	// WARNING - Do not return from this function without unlocking!
	visMutexLock();
	{
		// Abort if there is no camera.
		if (!Camera::current->isValid()) {
			goto visFrame_exit;
		}
		
		// Get the raw image from the camera
		{
			QSize size = Camera::current->getSize();
			
			Pixel* rgb = Camera::current->getRGB();
				visRaw.copyFrom(size.width(), size.height(), rgb);
			Camera::current->unlock();
		}
		

		
		/* Do vision processing */
		{
			/* Precalculate commonly used information */
			// generate all useful views (lines / path)
			// all vision functions depend on this one !!!!!!!!!!!!!!
			visClassifyPixelsByColor();

			/*  needed debugging views */
			{
				/* Fast debugging views */
				visCreateRedMinusGreenView();
				visCreateHSBViews();		// depends on visClassifyPixelsByColor()
				visCreateHSLViews();		// depends on visClassifyPixelsByColor()
				/* Slow debugging views */
				visCreateWhiteConditionView();
			}			
			
			/* essential pre-processing for visNavigationParams */
			{
				//use paulBlob to segment colored barrels
				//also modify paulBlob by setting all barrels found to orange
				blankColredBarrels();
				
				//this is called to update paulBlob, so visNavigationParams
				// can be annotated correctly
				updatePixelColors();
				
				/* get path plan view */
				// depends on visClassifyPixelsByColor()!!
				visGenPath();
			}
		
			/* init drawing of navigation colors,
				done here so visPathControlMorots can
				draw to this image too */
			visNavigationParams.copyFrom(paulBlob);			

			/*	find next goal for robot, and decide whether to use
				path planning, or sweeping lines mode 
				for driving motors 	*/
				// depends on visClassifyPixelsByColor()!!
			goal = robotWidthScan();	// returns -1 on error
			if(goal.y!=-1 && goal.y<visRaw.height-closenessThresh){
/*
				printf("width %d height %d	start(%d,%d)	goal(%d,%d)\n",
							visPathView.width, 								
							visPathView.height,								
							visPathView.width/2, 							
							visPathView.height-ROBOT_WIDTH, 							
							goal.x, 										
							goal.y);
							
				unsigned char * temp=(unsigned char *)visPathView.data;
				navigate(	temp, 				
							visPathView.width, 								
							visPathView.height,								
							visPathView.width/2, 							
							visPathView.height-ROBOT_WIDTH, 							
							goal.x, 										
							goal.y,
							30,
							ROBOT_WIDTH,
							ROBOT_WIDTH);
							
				printf("new goal(%d,%d)\n",							
							goal.x, 										
							goal.y);				
*/							
				//drive robot with path planning
				//visPathControlMotors(goal);
					//remove me
					visPlotNavigationParams();	// depends on visClassifyPixelsByColor()
											
			}
			else{
				//drive robot with close vision only
				visPlotNavigationParams();	// depends on visClassifyPixelsByColor()
					//remove me
					//visPathControlMotors(goal);
			}
			
			


		}
	}
	visFrame_exit:
	// Unlock the vision mutex.
	// This must always be done before this function returns.
	visMutexUnlock();
}

