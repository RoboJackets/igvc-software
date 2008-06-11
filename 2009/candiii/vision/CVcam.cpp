#include "CVcam.h"

CVcam::CVcam()
{
	capture = 0;
}

CVcam::~CVcam()
{
	if( capture ){
		cvReleaseCapture( &capture );
	}
}

int CVcam::connect(int deviceID, const char* filename)
{
	if(filename==NULL)
		capture = cvCaptureFromCAM( deviceID );
	else
		capture = cvCreateFileCapture( filename );
	return 1;
}

int CVcam::testwebcam()
{    
	IplImage* image;
	
    if( capture )
    {
        cvNamedWindow( "image", 1 );
        
        for(;;)
        {
            if( !cvGrabFrame( capture ))
                break;
            image = cvRetrieveFrame( capture );

            if( image )
            {
				cvShowImage( "image", image );
            }       

            if( cvWaitKey(10) >= 0 )
                break;
        }
        
        cvDestroyWindow( "image" );
    }
    
    

    return 1;

}
