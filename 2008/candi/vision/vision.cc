#include "vision.h"
#include "vision_color.h"
#include "vision_barrels.h"
#include "vision_navigation.h"
#include "vision/vision_line_blobber.h"

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

// DEBUG
#include "vision_util.h"

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
			visClassifyPixelsByColor();
			
			/* Fast debugging views */
			visCreateRedMinusGreenView();
			visCreateHSBViews();		// depends on visClassifyPixelsByColor()
			visCreateHSLViews();		// depends on visClassifyPixelsByColor()
			
			/* Execute primary analyses */
			visFindBarrels();		// depends on visClassifyPixelsByColor()
			visPlotNavigationParams();	// depends on visClassifyPixelsByColor()
			
			
			
			/* Slow debugging views */
			{
				
				// PERF: This view significantly reduces framerate!
				//       (35 fps -> 20 fps)
				{
					//visCreateWhiteCalibrationViewFromColor(Pixel(0, 255, 0));	// green
					
					static unsigned long hueTimeCounter = 0;
					hueTimeCounter++;
					
					/*
					u8 curHue = (u8) ((hueTimeCounter / 1) % 256);
					//printf("DEBUG: curHue=%d\n", (int) curHue);
					visCreateWhiteCalibrationViewUsingHue( curHue );
					*/
					
					/*
					u8 curHue = (u8) ((hueTimeCounter / 1) % 100);
					visCreateHSLColorSpaceView( curHue );	// PERF: 35 fps -> 20 fps
					*/
				}
				
				visCreateWhiteConditionView();
				
				//visTestRGBtoHSLConversions();
			}
			
			//HSL debug_yellow = RGBtoHSL(Pixel(255, 255, 0));
			//printf("DEBUG: H=%d, S=%d, L=%d\n", debug_yellow.hue, debug_yellow.saturation, debug_yellow.lightness);
			
			// TEMPORARY: Annotate pixel colors only
			visTestViewContent.copyFrom(visRaw);
			visAnnotatePixelColors(visTestViewContent);
		}
	}
visFrame_exit:
	// Unlock the vision mutex.
	// This must always be done before this function returns.
	visMutexUnlock();
}

