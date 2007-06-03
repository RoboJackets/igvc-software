/* 
 *  This file contains defines to convert global input variables
 *  into PjMat's internal abbreviated format.
 *  DO NOT INCLUDE THIS FILE ANYWHERE EXCEPT PjMat or clones!
 *
 */
 
 
/*
 *volatile double u=.4;volatile double v=.4;	// hfov/2 and vfov/2 respectively
 *volatile double h=1;					// height of camera
 *volatile double x=-0.8; volatile double y=03.14159;volatile  double z=3.14159;	//euler rotations about given axes
 */
#define u HFOV/2
#define v VFOV/2
#define h CAM_HEIGHT
#define x CAM_X_ROTATION
#define y CAM_Y_ROTATION
#define z CAM_Z_ROTATION

