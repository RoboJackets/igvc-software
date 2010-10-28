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


class ImageBufferManager
{
friend class Vision;
friend class CVcam;
friend class GuppyCam;
protected:
	ImageBufferManager(){};
	~ImageBufferManager(){};
	ImageBufferManager(const ImageBufferManager &);
	ImageBufferManager & operator=(const ImageBufferManager &);
	
public:
	static ImageBufferManager &getInstance();
	void init();
	void releaseAllImages();
	IplImage* getvisCvRaw();
	void setvisCvRaw(IplImage *);
	IplImage* getvisCvDebug();
	IplImage* getvisCvRedChannel();
	IplImage* getvisCvGreenChannel();
	IplImage* getvisCvBlueChannel();
	IplImage* getvisCvHSV();
	IplImage* getvisCvHue();
	IplImage* getvisCvSaturation();
	IplImage* getvisCvGrey();
	IplImage* getvisCvThresh();
	IplImage* getvisCvPath();
	IplImage* getvisCvHSVSmall();

	IplImage* getvisCvAdapt();
	IplImage* getvisCvAdaptSmall();
	
	IplImage* getvisCvGreyBig();
	IplImage* getvisCvRawTransform();
	IplImage* getvisCvGlutMask();
	void setvisCvGlutMask(IplImage*);
	IplImage* getvisCvRawTransformSmall();
	
	IplImage* getvisCvRamp();
	void setvisCvRamp(IplImage *);
	IplImage* getvisCvRampLines();
	void setvisCvRampLines(IplImage *);
	
	
private:

	
// NOTE: these MUST be initialized in "Robot::init()"
// EDFU : moved initialization to vision::init
 IplImage* visCvRaw;
 IplImage* visCvDebug;
 IplImage* visCvRedChannel;
 IplImage* visCvGreenChannel;
 IplImage* visCvBlueChannel;
 IplImage* visCvHSV;
 IplImage* visCvHue;
 IplImage* visCvSaturation;
 IplImage* visCvGrey;
 IplImage* visCvThresh;
 IplImage* visCvPath;
 IplImage* visCvHSVSmall;

 IplImage* visCvAdapt;
 IplImage* visCvAdaptSmall;

 IplImage* visCvGreyBig;
 IplImage* visCvRawTransform;
 IplImage* visCvGlutMask;
 IplImage* visCvRawTransformSmall;

 IplImage* visCvRamp;
 IplImage* visCvRampLines;

};
/* conversions ***********************************************/




#endif /*IMAGE_BUFFERS_H_*/
