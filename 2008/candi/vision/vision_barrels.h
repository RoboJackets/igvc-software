#ifndef _VISION_BARRELS_H_
#define _VISION_BARRELS_H_

#include "Pixel.h"
#include "Buffer2D.h"
#include "Rectangle.h"
#include <QList>

// OUTPUT: Original camera image annotated with barrel bounds/stats
extern Buffer2D<Pixel> visBarrels;

// OUTPUT: List of identified barrel bounds in the image
extern QList< Rectangle<int> > barrelBoundList;

// OUTPUT: Flags the pixels of barrels that have been identified
// 
// This information is used:
// 1. to prevent the same barrel from being identified multiple times
// 2. by other vision algorithms for various purposes
extern Buffer2D<bool> pixelIsWithinBarrel;

/**
 * Locates barrels within the current camera image.
 * 
 * Inputs from the global variables:
 * - 'visRaw'
 * - 'pixelIsOrange' - computed externally
 * - 'pixelIsWhite' - computed externally
 * 
 * Output is stored into the global variables:
 * - 'barrelBoundList'
 * - 'visBarrels'
 */
void visFindBarrels(void);

/**
 * Annotates 'imageToAnnotate' with the set of barrel bounds
 * identified by the most recent call to 'visFindBarrels'.
 * 
 * It is assumed that the dimensions of 'imageToAnnotate'
 * match the dimensions of 'visRaw'.
 * 
 * If 'showBarrelFinderParameters' is TRUE, then additional
 * parameters related to the barrel-finder algorithm will
 * also be drawn.
 */
void visAnnotateBarrelBounds(Buffer2D<Pixel>& imageToAnnotate, bool showBarrelFinderParameters);

#endif // _VISION_BARRELS_H_
