#include "Bumblebee2.h"

//#include <dc1394/conversions.h>
#include <common/logger/logger.h>

using namespace FlyCapture2;
using namespace cv;

Bumblebee2::Bumblebee2(string fileName): frameCount(0), frameLock(), _images(), _cam()
{
  FileStorage fs(fileName, FileStorage::READ); // Read the settings
  fs["Camera_Matrix"] >> _cameraMatrix;
  fs["Distortion_Coefficients"] >> _distCoeffs;
  if ( StartCamera() != 0 )
      Logger::Log(LogLevel::Error, "Camera failed to initialize.");
}

int Bumblebee2::StartCamera()
{
    const Mode k_fmt7Mode = MODE_3;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW16;
    Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    if ( numCameras < 1 )
    {
        Logger::Log(LogLevel::Error, "[Bumblebee2] No camera connected.");
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
        return -1;
    }

    // Set the settings to the camera
    error = _cam.SetFormat7Configuration(
        &fmt7ImageSettings,
        fmt7PacketInfo.recommendedBytesPerPacket>>1); //packet size of recommendedBytesPerPacket>>1(which is 2048) ensures proper transmission of data at 10fps, do not change

    unsigned int dis;
    float dat;
    _cam.GetFormat7Configuration(&fmt7ImageSettings, &dis, &dat);

    std::cout << "Packet Size set to " << dis <<" should be "<< (fmt7PacketInfo.recommendedBytesPerPacket>>1)<< std::endl;

    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }


    //Set Frame Rate
    Property frmRate;
    frmRate.type = FRAME_RATE;

    error = _cam.GetProperty( &frmRate );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    printf( "Frame rate is %3.2f fps\n", frmRate.absValue );

    // Start capturing images
    error = _cam.StartCapture(&ProcessFrame, this);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    return 0;
}

int Bumblebee2::CloseCamera()
{
    if(_cam.IsConnected())
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
    return 0;
}


void Bumblebee2::ptgrey2opencv(FlyCapture2::Image& img, cv::Mat& mat)
{
    //int stride = img.GetStride();
    //Mat newMat = cv::Mat(img.GetRows(), img.GetCols(), CV_8UC3, img.GetData(), stride);
    Mat newMat = cv::Mat(img.GetRows(), img.GetCols(), CV_8UC3, img.GetData(), cv::Mat::AUTO_STEP);

    mat = newMat.clone(); //must be cloned, otherwise repeated calls crush each other
    return;
}

ImageData Bumblebee2::left()
{
  return _images.left();
}

ImageData Bumblebee2::right()
{
  return _images.right();
}


Mat& Bumblebee2::LeftMat()
{
    return _images.leftMat();
}

Mat& Bumblebee2::RightMat()
{
    return _images.rightMat();
}

/*
void Bumblebee2::LockImages()
{
    _imagesLock.lock();
}

void Bumblebee2::UnlockImages()
{
    _imagesLock.unlock();
}
*/

StereoImageData Bumblebee2::Images()
{
    return _images;
}

FlyCapture2::Camera& Bumblebee2::Cam(void)
{
    return _cam;
}


Bumblebee2::~Bumblebee2()
{
    CloseCamera();
}


void ProcessFrame(Image* rawImage, const void* that)
{
    //Image* fake;
    Bumblebee2&  thisHere= *((Bumblebee2*)that);
    thisHere.frameLock.lock();
    Image savedRaw;
    Error error;
    Mat right, left;

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


    // Create a converted image
    Image convertedImage;

    // Convert the raw image
    error = savedRaw.Convert( PIXEL_FORMAT_BGR, &convertedImage );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return;
    }

    Bumblebee2::ptgrey2opencv(convertedImage,left);

    unsigned char* data = savedRaw.GetData();
    for(unsigned int i = 1; i < savedRaw.GetDataSize(); i += 2)
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

    Bumblebee2::ptgrey2opencv(convertedImage, right);


    thisHere.LockImages();
    Mat leftCorrected, rightCorrected;

    //StereoImageData newFrame(left, right);
    leftCorrected = thisHere.correctImage(left);
    rightCorrected = thisHere.correctImage(right);
    thisHere.Images(leftCorrected, rightCorrected);
    thisHere.UnlockImages();
    thisHere.frameCount++;
    thisHere.onNewData(thisHere.Images());
    //thisHere.onNewData(newFrame);
    return;
}


void Bumblebee2::Images(Mat& l, Mat& r)
{
  _images = StereoImageData(l,r);

}

void PrintError( FlyCapture2::Error error )
{
    std::stringstream msg;
    msg << "[Bumblebee2] ";
    msg << error.GetDescription();
    Logger::Log(LogLevel::Error, msg.str());
//    error.PrintErrorTrace();
}


Mat Bumblebee2::correctImage(Mat rawImg)
{
  return correctDistortion(rawImg, _cameraMatrix, _distCoeffs);
}
