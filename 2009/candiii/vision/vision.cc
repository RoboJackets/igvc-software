#include "vision.h"
#include "Pixel.h"
#include "Buffer2D.h"
#include <math.h>

#if USE_VISION_MUTEX
QMutex visMutex;
#endif

// ### BUFFERS ###
// Primary image buffer
Buffer2D<Pixel> visRaw;



// Performs all vision processing.
void visProcessFrame()
{
	// Lock the vision mutex before doing anything else.
	// WARNING - Do not return from this function without unlocking!
	visMutexLock();
	{
		// Abort if there is no camera.
		if ( 0 ) {
			goto visFrame_exit;
		}
		
		// Get the raw image from the camera
		{
			
		}
		
		/* Do vision processing */
		{
			
		}
		
		
	}
	visFrame_exit:
	// Unlock the vision mutex.
	// This must always be done before this function returns.
	visMutexUnlock();
}

