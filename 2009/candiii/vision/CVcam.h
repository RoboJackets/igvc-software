#ifndef CVCAM_H_
#define CVCAM_H_


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>


class CVcam
{
public:
	CVcam();
	virtual ~CVcam();
	CvCapture* capture;
	int testwebcam();
	int connect(int deviceID, const char* filename = NULL);
};

#endif /*CVCAM_H_*/
