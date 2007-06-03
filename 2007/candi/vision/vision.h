#ifndef _VISION_H_
#define _VISION_H_

#define USE_VISION_MUTEX 0

#include "Pixel.h"
#include "Buffer2D.h"

#if USE_VISION_MUTEX
#	include <qmutex.h>
#endif

/*
 * This file-class contains the robot's primary vision processing code.
 */

// This is called each frame to do vision processing
void visFrame();

// -----------------------------------------------------------------------------

// This mutex is locked while the image buffers (below) are being modified.
// Any GUI code which needs them (i.e. VideoViews) should lock this mutex
// while reading data.
#if USE_VISION_MUTEX
	extern QMutex visMutex;
	#define visMutexLock()		visMutex.lock()
	#define visMutexUnlock()	visMutex.unlock()
#else
	#define visMutexLock()
	#define visMutexUnlock()
#endif

// -----------------------------------------------------------------------------
// IMAGE BUFFERS

// In general, accesses to these buffers must be synchronized
// with the "visMutex" lock.

// Primary image buffer
extern Buffer2D<Pixel> visRaw;

// Auxilary testing view (for debugging purposes)
extern Buffer2D<Pixel> visTestViewContent;

#endif // _VISION_H_

