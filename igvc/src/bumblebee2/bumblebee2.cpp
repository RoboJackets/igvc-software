#include "bumblebee2.h"
#include <iostream>
#include <ros/console.h>

using namespace std;
using namespace FlyCapture2;

Bumblebee2::Bumblebee2()
{
    ROS_INFO_STREAM("WTF IS HAPPENING");
    if(startCamera() != 0)
    {
        ROS_ERROR_STREAM("Bumblebee2 failed to initialize.");
        cout << "PROBLEM" << endl;
    }
}

Bumblebee2::~Bumblebee2()
{
    closeCamera();
}

int Bumblebee2::startCamera()
{
    constexpr Mode k_fmt7Mode = MODE_3;
    constexpr PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW16;
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
        ROS_ERROR_STREAM("No camera connected.");
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

int Bumblebee2::closeCamera()
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

void Bumblebee2::PrintError(const Error& error)
{
    ROS_ERROR_STREAM(error.GetDescription());
}

void Bumblebee2::ProcessFrame(FlyCapture2::Image* rawImage, const void* callbackData)
{
    ROS_INFO_STREAM("ProcessFrame called.");
    cout << "PFRAME" << endl;
}
