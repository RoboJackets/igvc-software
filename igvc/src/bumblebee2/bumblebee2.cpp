#include "bumblebee2.h"
#include <exception>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

using namespace std;
using namespace FlyCapture2;
using namespace ros;

Bumblebee2::Bumblebee2(NodeHandle &handle)
    : _it(handle), cameraManagerLeft(handle, "bumblebee2/left", "file:///home/alchaussee/Desktop/igvc/src/igvc-software/sandbox/bumblebee2/camera_calib_left.yml"),
      cameraManagerRight(handle, "bumblebee2/right", "file:///home/alchaussee/Desktop/igvc/src/igvc-software/sandbox/bumblebee2/camera_calib_right.yml")
{
    try
    {
        startCamera();
    } catch(const char* s) {
        ROS_ERROR_STREAM("Bumblebee2 failed to initialize.");
        ROS_ERROR_STREAM(s);
        return;
    }

    _left_pub = _it.advertise("/stereo/left/image_raw", 1);
    _right_pub = _it.advertise("/stereo/right/image_raw", 1);
    _leftInfo_pub = handle.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1);
    _rightInfo_pub = handle.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1);
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

bool Bumblebee2::isOpen()
{
    return _cam.IsConnected();
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

    ROS_INFO_STREAM("Packet Size set to " << dis << " should be " << (fmt7PacketInfo.recommendedBytesPerPacket>>1));

    if (error != PGRERROR_OK)
        throw error.GetDescription();


    //Set Frame Rate
    Property frmRate;
    frmRate.type = FRAME_RATE;
    frmRate.absValue = 20;
    error = _cam.SetProperty(&frmRate);
    if (error != PGRERROR_OK)
        throw error.GetDescription();
    ROS_INFO_STREAM("Frame rate is " << frmRate.absValue << " fps.");

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
    //Image* fake;
    Bumblebee2&  self = *((Bumblebee2*)callbackData);
    
    Image savedRaw;
    Error error;

    error = savedRaw.DeepCopy((const Image*) rawImage);
    if (error != PGRERROR_OK)
        throw error.GetDescription();

    // Get the raw image dimensions
    PixelFormat pixFormat;
    unsigned int rows, cols, stride;
    savedRaw.GetDimensions( &rows, &cols, &stride, &pixFormat );

    // Create a converted image
    Image convertedImage;

    // Convert the raw image
    error = savedRaw.Convert( PIXEL_FORMAT_BGR, &convertedImage );
    if (error != PGRERROR_OK)
        throw error.GetDescription();

    // convertedImage is now the right image.

    sensor_msgs::Image right;
    right.header.stamp = ros::Time::now();
    right.header.frame_id = "/camera_right";
    right.height = convertedImage.GetRows();
    right.width = convertedImage.GetCols();
    right.encoding = sensor_msgs::image_encodings::BGR8;
    right.step = convertedImage.GetStride();
    right.data.reserve(convertedImage.GetDataSize());
    for(int i = 0; i < convertedImage.GetDataSize(); i++)
        right.data.push_back(convertedImage.GetData()[i]);

    unsigned char* data = savedRaw.GetData();
    for(unsigned int i = 1; i < savedRaw.GetDataSize(); i += 2)
        swap(data[i], data[i-1]);

    error = savedRaw.Convert( PIXEL_FORMAT_BGR, &convertedImage );
    if (error != PGRERROR_OK)
        throw error.GetDescription();

    // convertedImage is now the left image.

    sensor_msgs::Image left;
    left.header.stamp = right.header.stamp;
    left.header.frame_id = "/camera_left";
    left.height = convertedImage.GetRows();
    left.width = convertedImage.GetCols();
    left.encoding = sensor_msgs::image_encodings::BGR8;
    left.step = convertedImage.GetStride();
    left.data.reserve(convertedImage.GetDataSize());
    for(int i = 0; i < convertedImage.GetDataSize(); i++)
        left.data.push_back(convertedImage.GetData()[i]);
        
    sensor_msgs::CameraInfo cl = self.cameraManagerLeft.getCameraInfo();
    cl.header.frame_id = "/camera_left";
    cl.header.stamp = right.header.stamp;
    self.cameraManagerLeft.setCameraInfo(cl);

    sensor_msgs::CameraInfo cr = self.cameraManagerRight.getCameraInfo();
    cr.header.frame_id = "/camera_right";
    cr.header.stamp = right.header.stamp;
    self.cameraManagerRight.setCameraInfo(cr);

    self._left_pub.publish(left);
    self._leftInfo_pub.publish(self.cameraManagerLeft.getCameraInfo());
    self._right_pub.publish(right);
    self._rightInfo_pub.publish(self.cameraManagerRight.getCameraInfo());

    return;
}
