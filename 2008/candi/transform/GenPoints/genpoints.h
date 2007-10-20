#ifndef genpoints_h
#define genpoints_h

/*
 *  genpoints.h
 *  
 *
 *  Created by Paul Foster on Wed May 09 2007.
 *	Ported from MATLAB to allow regenerating points on the fly.
 *
 *
 */

#include <stdio.h>
#include <math.h>
#include "../pglobals.h"

/*	genpoints([width], [height],[focal length],[divisor])
 *	focal length can be computed by:
 *	f=[width in pixels]/[hfov]
 *
 *	divisor MUST be a common divisor of width and height, heigher values increase speed
 *	at the expense of accuracy
 */
 
 /* must have defined: 
  * global: double FINAL_SCALE_FACTOR	//(put final image in proper units)
  *			double HFOV
  *			double SCALE_FREE_FOCAL_LENGTH = 1/HFOV
  *			int TEX_DIM=[power of 2 greater than width and greater than height]
  *			double ASPECT_RATIO			//MUST be >1 
  */
void genpoints(int divisor);

#endif

