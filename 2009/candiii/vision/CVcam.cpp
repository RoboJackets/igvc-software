#include "CVcam.h"
#include "main.h"
#include "image_buffers.h"


CVcam::CVcam()
{
	capture = 0;
	camconnected = 0;
}

CVcam::~CVcam()
{
	if ( capture )
	{
		cvReleaseCapture( &capture );
	}
}

int CVcam::connect(int deviceID, const char* filename)
{
	if (filename==NULL)
	{
		capture = cvCaptureFromCAM( deviceID );
	}
	else
	{
		capture = cvCreateFileCapture( filename );
	}
	if (capture)
	{
		camconnected=1;
		return 1;
	}
	else
	{
		camconnected=0;
		//return 0;
		printf("Error connecting to camera \n");
		exit(-1);
	}
}

int CVcam::isValid()
{
	return camconnected;
}

int CVcam::GrabCvImage()
{
	if (!cvGrabFrame( capture ))
		return 0;

	// get and set
	visCvRaw = cvRetrieveFrame( capture );

	return 1;
}



//void* image_grab_loop_caller(void* arg) {
//    static_cast<CVcam*>(arg)->testwebcamandconvertloop();
//    return NULL;
//}
//
//void CVcam::startImageGrabThread(void* obj) {
//    pthread_t grabImageThread;
//    pthread_create(&grabImageThread, NULL, image_grab_loop_caller, obj);
//}
//
//int CVcam::testwebcamandconvertloop() {
//    if ( capture ) {
//        GrabCvImage();
//        visRaw.resize(visCvRaw->width,visCvRaw->height);
//        //image grab loop
//        //loop because we're in a thread
//        for (;;) {
//            GrabCvImage();
//            if ( visCvRaw ) {
//                //convert usb camera IplImage to our Buffer2D<Pixel> format
//                IplToBuff2D(visCvRaw,visRaw);
//            }
//        }
//    }
//    return 1;
//}


void CVcam::loadSettings()
{
	// currently no settings for USB camera
	printf("No USB Camera settings available \n");
}

