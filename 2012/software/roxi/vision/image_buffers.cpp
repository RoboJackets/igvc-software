#include "image_buffers.h"


/* buffers ***********************************************/
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

IplImage* pfThresh;

/* conversions ***********************************************/

/* singleton *************************************************/
ImageBufferManager& ImageBufferManager::getInstance()
{
	static ImageBufferManager instance;
	return instance;
}

/* init ******************************************************/
void ImageBufferManager::init()
{
		/* 3 plane images (640x480) */
		//visCvDebug = cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 3);
		visCvHSV = cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 3);
		visCvRawTransform = cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 3);

		/* 1 plane images (640x480) */
		visCvAdapt = cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 1);
		visCvGreyBig = cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 1);
		pfThresh = cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 1);//not really 640x480

		/* 3 plane images (320x240) */
		visCvHSVSmall = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 3);
		visCvRawTransformSmall = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 3);
		visCvDebug = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 3);

		/* 1 plane images (320x240) */
		visCvHue = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
		visCvSaturation = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
		visCvGrey = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
		visCvThresh = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
		visCvPath = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
		visCvAdaptSmall = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
		
		
		
}

void ImageBufferManager::releaseAllImages()
{
	/* free all in image_buffers.h */
	if ( visCvRaw !=NULL) cvReleaseImage(&visCvRaw );
	if ( visCvDebug !=NULL) cvReleaseImage(&visCvDebug );
	if ( visCvRedChannel !=NULL) cvReleaseImage(&visCvRedChannel );
	if ( visCvGreenChannel !=NULL) cvReleaseImage(&visCvGreenChannel );
	if ( visCvBlueChannel !=NULL) cvReleaseImage(&visCvBlueChannel );
	if ( visCvHSV !=NULL) cvReleaseImage(&visCvHSV );
	if ( visCvHue !=NULL) cvReleaseImage(&visCvHue );
	if ( visCvSaturation !=NULL) cvReleaseImage(&visCvSaturation );
	if ( visCvGrey !=NULL) cvReleaseImage(&visCvGrey );
	if ( visCvThresh !=NULL) cvReleaseImage(&visCvThresh );
	if ( visCvPath !=NULL) cvReleaseImage(&visCvPath );
	if ( visCvHSVSmall !=NULL) cvReleaseImage(&visCvHSVSmall );
	if ( visCvAdapt !=NULL) cvReleaseImage(&visCvAdapt );
	if ( visCvAdaptSmall !=NULL) cvReleaseImage(&visCvAdaptSmall );
	if ( visCvGreyBig !=NULL) cvReleaseImage(&visCvGreyBig );
	if ( visCvRawTransform !=NULL) cvReleaseImage(&visCvRawTransform );
	if ( visCvGlutMask !=NULL) cvReleaseImage(&visCvGlutMask );
	if ( visCvRawTransformSmall !=NULL) cvReleaseImage(&visCvRawTransformSmall );

	if ( visCvRamp !=NULL) cvReleaseImage(&visCvRamp );
	if ( visCvRampLines !=NULL) cvReleaseImage(&visCvRampLines );
	if ( pfThresh !=NULL) cvReleaseImage(&pfThresh );

}
/* getters ***************************************************/

IplImage* ImageBufferManager::getvisCvRaw()
{
	return visCvRaw;
}
void ImageBufferManager::setvisCvRaw(IplImage* image)
{
	visCvRaw = image;
}
IplImage* ImageBufferManager::getvisCvDebug()
{
	return visCvDebug;
}

IplImage* ImageBufferManager::getvisCvRedChannel()
{
	return visCvRedChannel;
}

IplImage* ImageBufferManager::getvisCvGreenChannel()
{
	return visCvGreenChannel;
}

IplImage* ImageBufferManager::getvisCvBlueChannel()
{
	return visCvBlueChannel;
}

IplImage* ImageBufferManager::getvisCvHSV()
{
	return visCvHSV;
}

IplImage* ImageBufferManager::getvisCvHue()
{
	return visCvHue;
}

IplImage* ImageBufferManager::getvisCvSaturation()
{
	return visCvSaturation;
}

IplImage* ImageBufferManager::getvisCvGrey()
{
	return visCvGrey;
}

IplImage* ImageBufferManager::getvisCvThresh()
{
	return visCvThresh;
}

IplImage* ImageBufferManager::getvisCvPath()
{
	return visCvPath;
}

IplImage* ImageBufferManager::getvisCvHSVSmall()
{
	return visCvHSVSmall;
}

IplImage* ImageBufferManager::getvisCvAdapt()
{
	return visCvAdapt;
}

IplImage* ImageBufferManager::getvisCvAdaptSmall()
{
	return visCvAdaptSmall;
}
	
IplImage* ImageBufferManager::getvisCvGreyBig()
{
	return visCvGreyBig;
}

IplImage* ImageBufferManager::getvisCvRawTransform()
{
	return visCvRawTransform;
}

IplImage* ImageBufferManager::getvisCvGlutMask()
{
	return visCvGlutMask;
}

void ImageBufferManager::setvisCvGlutMask(IplImage* image)
{
	visCvGlutMask = image;
}
IplImage* ImageBufferManager::getvisCvRawTransformSmall()
{
	return visCvRawTransformSmall;
}

IplImage* ImageBufferManager::getvisCvRamp()
{
	return visCvRamp;
}

void ImageBufferManager::setvisCvRamp(IplImage* image)
{
	visCvRamp = image;
}

IplImage* ImageBufferManager::getvisCvRampLines()
{
	return visCvRampLines;
}

void ImageBufferManager::setvisCvRampLines(IplImage* image)
{
	visCvRampLines = image;
}

IplImage* ImageBufferManager::getpfThresh()
{
	return pfThresh;
}

void ImageBufferManager::setpfThresh(IplImage* image)
{
	cvReleaseImage(&pfThresh);
	pfThresh = image;
}
