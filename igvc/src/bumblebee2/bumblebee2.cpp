#include "bumblebee2.h"
#include <exception>

using namespace std;
using namespace FlyCapture2;

Bumblebee2::Bumblebee2()
{
    try
    {
        startCamera();
    } catch(const char* s) {
        ROS_ERROR_STREAM("Bumblebee2 failed to initialize.");
        ROS_ERROR_STREAM(s);
    }
}

Bumblebee2::~Bumblebee2()
{
    try
    {
        closeCamera();
    } catch(const char* s) {
        ROS_ERROR_STREAM("Bumblebee2 failed to close.");
        ROS_ERROR_STREAM(s); 
    }
}

void Bumblebee2::startCamera()
{
    constexpr Mode k_fmt7Mode = MODE_3;
    constexpr PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW16;
    Error error;
    
    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
        throw error.GetDescription();

    if ( numCameras < 1 )
        throw "No camera connected.";

    PGRGuid guid;
    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
        throw error.GetDescription();

    // Connect to a camera
    error = _cam.Connect(&guid);
    if (error != PGRERROR_OK)
        throw error.GetDescription();

    // Query for available Format 7 modes
    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = k_fmt7Mode;
    error = _cam.GetFormat7Info( &fmt7Info, &supported );
    if (error != PGRERROR_OK)
        throw error.GetDescription();

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
        throw error.GetDescription();

    if ( !valid )
        throw "Invalid Settings.";

    // Set the settings to the camera
    error = _cam.SetFormat7Configuration(
        &fmt7ImageSettings,
        fmt7PacketInfo.recommendedBytesPerPacket>>1); //packet size of recommendedBytesPerPacket>>1(which is 2048) ensures proper transmission of data at 10fps, do not change

    unsigned int dis;
    float dat;
    _cam.GetFormat7Configuration(&fmt7ImageSettings, &dis, &dat);

    cout << "Packet Size set to " << dis <<" should be " << (fmt7PacketInfo.recommendedBytesPerPacket>>1)<< endl;

    if (error != PGRERROR_OK)
        throw error.GetDescription();


    //Set Frame Rate
    Property frmRate;
    frmRate.type = FRAME_RATE;

    error = _cam.GetProperty( &frmRate );
    if (error != PGRERROR_OK)
        throw error.GetDescription();
    cout << "Frame rate is " << frmRate.absValue << " fps." << endl;

    // Start capturing images
    error = _cam.StartCapture(&ProcessFrame, this);
    if (error != PGRERROR_OK)
        throw error.GetDescription();
}

void Bumblebee2::closeCamera()
{
    if(_cam.IsConnected())
    {
        Error error;
        // Stop capturing images
        error = _cam.StopCapture();
        if (error != PGRERROR_OK)
            throw error.GetDescription();

        // Disconnect the camera
        error = _cam.Disconnect();
        if (error != PGRERROR_OK)
            throw error.GetDescription();
    }
}

void Bumblebee2::ProcessFrame(FlyCapture2::Image* rawImage, const void* callbackData)
{
    ROS_INFO_STREAM("ProcessFrame called.");
    cout << "PFRAME" << endl;
}
