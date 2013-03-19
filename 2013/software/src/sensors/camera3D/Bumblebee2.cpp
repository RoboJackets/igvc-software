#include "Bumblebee2.h"
#include <stdio.h>
#include "sensors/camera3D/imgUtils.h"
#include <opencv2/highgui/highgui.hpp>
#include "sensors/camera3D/imgUtils.h"
#include <unistd.h>
//#include <dc1394/conversions.h>


using namespace FlyCapture2;
using namespace cv;

Bumblebee2::Bumblebee2(): _running(true), _images(), _cam(), frameCount(0), frameLock()
{
    StartCamera();
}

int Bumblebee2::StartCamera()
{
    const Mode k_fmt7Mode = MODE_3;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW16;
    Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    printf("Number of cameras detected: %u\n", numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    if ( numCameras < 1 )
    {
        printf( "Insufficient number of cameras... exiting\n" );
        return -1;
    }

    PGRGuid guid;
    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }


    // Connect to a camera
    error = _cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Query for available Format 7 modes
    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = k_fmt7Mode;
    error = _cam.GetFormat7Info( &fmt7Info, &supported );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

    bool valid;
    Format7PacketInfo fmt7PacketInfo;

    // Validate the settings to make sure that they are valid
    error = _cam.ValidateFormat7Settings(
        &fmt7ImageSettings,
        &valid,
        &fmt7PacketInfo );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    if ( !valid )
    {
        // Settings are not valid
		printf("Format7 settings are not valid\n");
        return -1;
    }

    // Set the settings to the camera

    error = _cam.SetFormat7Configuration(
        &fmt7ImageSettings,
        fmt7PacketInfo.recommendedBytesPerPacket );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Start capturing images
    error = _cam.StartCapture(&ProcessFrame, this);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
}

/*
void Bumblebee2::ProcessFrame(Image* rawImage, const void* otherThings)
{
        Image savedRaw;
        Error error;
        Mat right, left;
        error = savedRaw.DeepCopy((const Image*) &rawImage);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return;
        }

        // Get the raw image dimensions
        PixelFormat pixFormat;
        unsigned int rows, cols, stride;
        savedRaw.GetDimensions( &rows, &cols, &stride, &pixFormat );

	// Create a converted image
        Image convertedImage;

        // Convert the raw image
        error = savedRaw.Convert( PIXEL_FORMAT_BGR, &convertedImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return;
        }

        ptgrey2opencv(convertedImage,right);

		unsigned char* data = savedRaw.GetData();
		for(int i = 1; i < savedRaw.GetDataSize(); i += 2)
		{
            char tmp = data[i];
			data[i] = data[i-1];
			data[i-1] = tmp;
		}

        error = savedRaw.Convert( PIXEL_FORMAT_BGR, &convertedImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return;
        }

        ptgrey2opencv(convertedImage, left);
        return;

}
*/

/*

int Bumblebee2::Run()
{
    Mat left;
    Mat right;
    Error error;

    // Retrieve frame rate property
    Property frmRate;
    frmRate.type = FRAME_RATE;
    error = _cam.GetProperty( &frmRate );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    //printf( "Frame rate is %3.2f fps\n", frmRate.absValue );

    Image rawImage;
    Image savedRaw;
    namedWindow( "leftDisp", CV_WINDOW_AUTOSIZE );
    namedWindow( "rightDisp", CV_WINDOW_AUTOSIZE );
    //trash first frame
    error = _cam.RetrieveBuffer((FlyCapture2::Image*) &rawImage);

    while(1)
    {
        // Retrieve an image
        error = _cam.RetrieveBuffer((FlyCapture2::Image*) &rawImage);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            continue;
        }
        error = savedRaw.DeepCopy((const Image*) &rawImage);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            continue;
        }


        // Get the raw image dimensions
        PixelFormat pixFormat;
        unsigned int rows, cols, stride;
        savedRaw.GetDimensions( &rows, &cols, &stride, &pixFormat );

	// Create a converted image
        Image convertedImage;

        // Convert the raw image
        error = savedRaw.Convert( PIXEL_FORMAT_BGR, &convertedImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }


        ptgrey2opencv(convertedImage,right);

		unsigned char* data = savedRaw.GetData();
		for(int i = 1; i < savedRaw.GetDataSize(); i += 2)
		{
            char tmp = data[i];
			data[i] = data[i-1];
			data[i-1] = tmp;
		}

        error = savedRaw.Convert( PIXEL_FORMAT_BGR, &convertedImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        ptgrey2opencv(convertedImage, left);
        //convertedImage.Save("/home/robojackets/Desktop/pgright.png");
        imshow( "leftDisp", left);
        imshow( "rightDisp", right);
        //imwrite("/home/robojackets/Desktop/right.png",right);
        //imwrite("/home/robojackets/Desktop/left.png",left);
        if(waitKey(30) >= 0) break;
    }

    CloseCamera();

	return 0;
}
*/
int Bumblebee2::CloseCamera()
{
    Error error;
    // Stop capturing images
    error = _cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Disconnect the camera
    error = _cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
}


void Bumblebee2::ptgrey2opencv(FlyCapture2::Image& img, cv::Mat& mat)
{
    int stride = img.GetStride();
    //Mat newMat = cv::Mat(img.GetRows(), img.GetCols(), CV_8UC3, img.GetData(), stride);
    Mat newMat = cv::Mat(img.GetRows(), img.GetCols(), CV_8UC3, img.GetData(), cv::Mat::AUTO_STEP);

    mat = newMat.clone(); //must be done, otherwise repeated calls crush each other
    return;
}

void Bumblebee2::Running(bool newState)
{
    _running = newState;
}

bool Bumblebee2::Running(void)
{
    return _running;
}

Mat& Bumblebee2::Left()
{
    return _images.RightImage();
}

Mat& Bumblebee2::Right()
{
    return _images.LeftImage();
}

void Bumblebee2::LockImages()
{
    _imagesLock.lock();
}

void Bumblebee2::UnlockImages()
{
    _imagesLock.unlock();
}

StereoPair Bumblebee2::Images()
{
    return _images;
}

FlyCapture2::Camera& Bumblebee2::Cam(void)
{
    return _cam;
}


Bumblebee2::~Bumblebee2()
{
    //dtor
}


void ProcessFrame(Image* rawImage, const void* that)
{
    Image* fake;
    Bumblebee2&  thisHere= *((Bumblebee2*)that);
    thisHere.frameLock.lock();
    Image savedRaw;
    Error error;
    Mat right, left;



    char filename[512];
    sprintf( filename, "./newthings/%u-%d.jpg", 121, thisHere.frameCount++);


    // Convert the raw image
    Image convImage;
    error = (*rawImage).Convert( PIXEL_FORMAT_BGRU, &convImage );
    error = convImage.Save( filename );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return;
    }



    error = savedRaw.DeepCopy((const Image*) rawImage);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return;
    }

    thisHere.frameLock.unlock();

    // Get the raw image dimensions
    PixelFormat pixFormat;
    unsigned int rows, cols, stride;
    savedRaw.GetDimensions( &rows, &cols, &stride, &pixFormat );

/*
//Begin experiment
    unsigned char* data = savedRaw.GetData();
    cout << "where" << endl;
    unsigned char* dest = new unsigned char[rows*cols*8];
    cout << "is" << endl;
    dc1394_deinterlace_stereo(data, dest, 2*cols, rows);
    cout << "it" << endl;
    Mat right = cv::Mat(rows, cols, CV_8UC1, dest, cols);
    cout << "dumped?" << endl;
    Mat left= cv::Mat(rows, cols, CV_8UC1, dest + ((2*rows*cols)>>1)-1, cols);
    //Mat left= cv::Mat(rows, cols, CV_8UC4, dest + ((rows*cols*4)>>1)-1, cv::Mat::AUTO_STEP);
//End experiment
*/



// Create a converted image
    Image convertedImage;

    // Convert the raw image
    error = savedRaw.Convert( PIXEL_FORMAT_BGR, &convertedImage );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return;
    }

/*
        char filename[512];
        sprintf( filename, "./newthings/%u-%d.bmp", 121, thisHere.frameCount++ );
    error = convertedImage.Save( filename );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return;
    }
*/


    Bumblebee2::ptgrey2opencv(convertedImage,right);

    unsigned char* data = savedRaw.GetData();
    for(int i = 1; i < savedRaw.GetDataSize(); i += 2)
    {
        char tmp = data[i];
        data[i] = data[i-1];
        data[i-1] = tmp;
    }

    error = savedRaw.Convert( PIXEL_FORMAT_BGR, &convertedImage );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return;
    }

    Bumblebee2::ptgrey2opencv(convertedImage, left);


    thisHere.LockImages();
    thisHere.Left() = left.clone();
    thisHere.Right() = right.clone();
    thisHere.UnlockImages();
    thisHere.onNewData(thisHere.Images());
    //delete[] dest;
    return;

}


void PrintError( FlyCapture2::Error error )
{
    error.PrintErrorTrace();
}


void dc1394_deinterlace_stereo(unsigned char* src, unsigned char* dest, int width, int height)
{
    register int i = (width*height)-1;
    register int j = ((width*height)>>1)-1;
    register int k = (width*height)-1;
    unsigned char filler;
    while (i >= 1) {
        dest[k--] = src[i--];
        //cout << "Source is the problem" << endl;
        dest[j--] = src[i--];
    }
    return;
}
