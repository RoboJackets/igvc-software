#ifndef IMAGE_BUFFERS_H_
#define IMAGE_BUFFERS_H_


#include <opencv/cv.h>
#include <opencv/highgui.h>

/* ALL IMAGE_BUFFERS GO HERE
 *
 * This file contains all the image buffers to be used (GLOBALLY)
 *   by: Chris McClanahan
 */


/* buffers ***********************************************/





// NOTE: these MUST be initialized in "Robot::init()"
extern IplImage* visCvRaw;
extern IplImage* visCvDebug;
extern IplImage* visCvRedChannel;
extern IplImage* visCvGreenChannel;
extern IplImage* visCvBlueChannel;
extern IplImage* visCvHSV;
extern IplImage* visCvHue;
extern IplImage* visCvSaturation;
extern IplImage* visCvGrey;
extern IplImage* visCvThresh;
extern IplImage* visCvPath;
extern IplImage* visCvHSVSmall;

extern IplImage* visCvAdapt;
extern IplImage* visCvAdaptSmall;

extern IplImage* visCvGreyBig;
extern IplImage* visCvRawTransform;
extern IplImage* visCvGlutMask;
extern IplImage* visCvRawTransformSmall;

extern IplImage* visCvRamp;
extern IplImage* visCvRampLines;


/* conversions ***********************************************/




#endif /*IMAGE_BUFFERS_H_*/
