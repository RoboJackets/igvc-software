/*
 *  pglobals.h
 *  Created by Paul Foster on Wed May 09 2007.
 */

/*****************
 *	Globals!!!   *
 *****************/

#ifndef pglobals_h
#define pglobals_h

/*****************
 *	 Constants	 *
 *****************/

#define FINAL_SCALE_FACTOR .69	/* Put final image in proper units (empirically determined) */
#define HFOV (.8602)             //.8602 empirical value ~=49 degrees (in radians)
   /* D-1 NTSC uses 720x486
    * Square NTSC uses 648/486 
    * The camera uses D-1 NTSC 
   */
#define VFOV (HFOV*480/648)		// adjust for non-square NTSC pixels [648 == 720*(9/10)]	
	//(HFOV*420/648)

#define SCALE_FREE_FOCAL_LENGTH  (1/HFOV)	//actually 1.16252ish
#define TEX_DIM (1024)				/* Power of 2 > TRANSFORM_INPUT_WIDTH */
#define ASPECT_RATIO (3/2)			/* MUST be >1 */

/* Defines the width/height of the input images to the perspective transform */
#define TRANSFORM_INPUT_WIDTH 720
#define TRANSFORM_INPUT_HEIGHT 480

/* (Must be a common factor of the width/height values above.
 * Higher values increase the speed of the perspective transform at the expense of accuracy.) */
#define TRANSFORM_INPUT_DIVISOR 10

/* Defines the width/height of the output images from the perspective transform */
// NOTE: the width should be divisible by 4 for grayscale and black&white views to work
// NOTE: the aspect ratio of the output should match the aspect ratio of the input
#define TRANSFORM_OUTPUT_WIDTH 456 //900
#define TRANSFORM_OUTPUT_HEIGHT 304 //600
//#define OUTPUT_SCALE_FACTOR =.1//remove and multiply in with FSF for marginal speed boost

/************************
 *    Input Variables   *
 ************************/
 
/* 
 * Camera Info Vars
 * 
 * USE_AUX_CAM_SENSORS must be defined for the camera info to work as inputs
 */
//#define USE_AUX_CAM_SENSORS
extern volatile double CAM_HEIGHT;					
extern volatile double CAM_X_ROTATION; 
extern volatile double CAM_Y_ROTATION;
extern volatile double CAM_Z_ROTATION;
 
 













#ifndef PGLOBALS_PROGRAM_STATIC//declared by caller to avoid conflicting variables
#define PGLOBALS_PROGRAM_STATIC

#define paulpedantic
/************************
 *	 State Variables    *
 ************************/
/* The texture */
int texture[1];
int currx;
int curry;
double xscale;
double yscale;







#ifdef PaulModeGo

#define IWIDTH		TRANSFORM_INPUT_WIDTH
#define IHEIGHT 	TRANSFORM_INPUT_HEIGHT
#define IDIVISOR	TRANSFORM_INPUT_DIVISOR
#define OWIDTH 		TRANSFORM_OUTPUT_WIDTH 
#define OHEIGHT 	TRANSFORM_OUTPUT_HEIGHT

#endif //PaulModeGo














#endif //PGLOBALS_PROGRAM_STATIC
#endif //pglobals_h
