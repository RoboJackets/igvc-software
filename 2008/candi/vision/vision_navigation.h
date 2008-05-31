#ifndef _VISION_NAVIGATION_H_
#define _VISION_NAVIGATION_H_

#include "Buffer2D.h"
#include "Pixel.h"
#include "MotorOutput.h"
#include "DriveMotion.h"
#include <math.h>

// OUTPUT: Visual representation of the navigation parameters.
extern Buffer2D<Pixel> visNavigationParams;

// OUTPUT: Trajectory picked by the vision-navigation algorithm
extern MotorOutput autonomousModeMotorOutput;

// Draws a graphical representation of the current navigation parameters.
void visPlotNavigationParams(void);

// ### PARAMETERS ###

// Number of paths that are assessed between the starting/ending angles
// 1 <= NAV_PATH__NUM
const int NAV_PATH__NUM = 30;

// Defines the "view/navigation cone", which is where the set of
// considered navigation paths is taken from.
const double NAV_PATH__VIEW_CONE__OFFSET = 45.0; //30;<-without-transform
const double NAV_PATH__VIEW_CONE__START_ANGLE = 0.0 + NAV_PATH__VIEW_CONE__OFFSET;	// >= 0.0
const double NAV_PATH__VIEW_CONE__END_ANGLE = 180.0 - NAV_PATH__VIEW_CONE__OFFSET;	// <= 180.0

const double NAV_PATH__VIEW_CONE__DELTA_ANGLE = NAV_PATH__VIEW_CONE__END_ANGLE - NAV_PATH__VIEW_CONE__START_ANGLE;
const double NAV_PATH__VIEW_CONE__SPACING = NAV_PATH__VIEW_CONE__DELTA_ANGLE / (NAV_PATH__NUM-1);

// XXX: Controls how many pixels *near* a path-pixel are searched for dangerous pixels
// 0 <= NAV_PATH__PATH_SEARCH_GIRTH
const int NAV_PATH__PATH_SEARCH_GIRTH = 5;

// (do not change without reason!)
const int NAV_PATH__CENTER_PATH_ID = (int) round((90.0 - NAV_PATH__VIEW_CONE__START_ANGLE) / NAV_PATH__VIEW_CONE__SPACING);

// Proportional to the lengths of the paths (in image space)
const double NAV_PATH__VIEW_DISTANCE_MULTIPLIER = 0.70; //1.00;<-without-transform		/* > 0.0 */

// Amount of danger posed by a single barrel-pixel
const int DANGER_PER_BARREL_PIXEL = 1;
// Amount of danger posed by a single line-pixel
const int DANGER_PER_LINE_PIXEL =5;// 4;

// Path danger values higher than this will be clipped to this value
const int MAX_PATH_DANGER = 50;					// >= 0

const int NAV_PATH__DANGER_SMOOTHING__RADIUS = 5;		// >= 0


#endif // _VISION_NAVIGATION_H_
