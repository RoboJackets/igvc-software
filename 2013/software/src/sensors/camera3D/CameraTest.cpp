#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>

//#include <triclops.h>

#include <flycapture/FlyCapture2.h>

#include <flycapture/Image.h>
#include <flycapture/Camera.h>
#include <flycapture/Error.h>
#include <flycapture/BusManager.h>
#include <flycapture/FlyCapture2Defs.h>


#include <opencv2/opencv.hpp>

using namespace std;
using namespace FlyCapture2;


void deInterleave(Image& Interleaved, Image& Right, Image& Left);
void PrintError( Error error );

main()
{

    //Set up Initial variables
    Error error;
    Camera cam;
    BusManager busMgr;

    PGRGuid guid;
    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
    {
       PrintError(error);
       return -1;
    }
    else
    {
        cout << "guid successfully acquired" << endl;
    }


     // Connect to a camera
     error = cam.Connect(&guid);
     if (error != PGRERROR_OK)
     {
         PrintError( error );
         return -1;
     }
     else
     {
         cout<< "Camera Successfully connected" << endl;
     }


    //Set Trigger Mode to External
    ca.SetTriggerMode
         error = cam.Connect(&guid);
     if (error != PGRERROR_OK)
     {
         PrintError( error );
         return -1;
     }
     else
     {
         cout<< "Trigger Mode Successfully Set" << endl;
     }



    bool supported;

    error = cam.SetVideoModeAndFrameRate(VIDEOMODE_FORMAT7, FRAMERATE_30);

   if ( error != PGRERROR_OK )
    {
        PrintError(error);
    }
    else
    {
        cout << "Correctly set Video Mode and Frame Rate";
    }

    Format7Info fmt7Info;
    fmt7Info.mode = MODE_3;
    fmt7Info.pixelFormatBitField = 0;

    error = cam.GetFormat7Info( &fmt7Info, &supported );
    if ( error != PGRERROR_OK )
    {
        PrintError(error);
    }
    else
    {
        cout << "Successfully queried Format7 info" << endl;
        cout << "Suppored value is" << supported;
    }
    /*
    //Structures for setting camera to appropriate mode for
    Format7ImageSettings cameraSettings;
    cameraSettings.pixelFormat = PIXEL_FORMAT_RAW16;
    cameraSettings.height = 1024;
    cameraSettings.width = 768;

    //cameraSettings.height = fmt7Info.maxHeight;
    //cameraSettings.width = fmt7Info.maxWidth;
    cout << "Max width is " << fmt7Info.maxWidth << endl;
    cout << "Max height is " << fmt7Info.maxHeight << endl;
    cameraSettings.offsetX = 0;
    cameraSettings.offsetY = 0;
    cameraSettings.mode = MODE_3;
    //(PIXEL_FORMAT_RAW16, 1024, MODE_3, 0, 0, PIXEL_FORMAT_RAW16, 0, 768);
    Format7PacketInfo packetInfo;
    bool goodSettings;


    //Get Camera Information from bus manager


    //Configure Camera to Proper Format and Mode for Stereo Camera
    error = cam.ValidateFormat7Settings(&cameraSettings, &goodSettings, &packetInfo);

    if(goodSettings)
    {

        error = cam.SetFormat7Configuration(&cameraSettings, packetInfo.recommendedBytesPerPacket);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            std::cout<<"Error setting Format7 Configuration of camera" << endl;
        }
        else
        {
            std::cout<< "Set Format7 config successfully" << endl;
        }
    }
    else
    {
        cout << "Format7 Settings were bad!" << endl;
        PrintError(error);

    }
    */
    cam.StartCapture();

    Image rawImage; //Interleaved bytes fed back by camera in format_7
    Image rawImageL; //Left Image from Camera's Perspective
    Image rawImageR; //Right Image from Camera's Perspective

    int i;
    sleep(10);
    for(i=0;i<1;i++)
    {

        error = cam.RetrieveBuffer( &rawImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            printf("Error retrieving image from buffer!\n");
        continue;
        }
        printf("RawImage has %d rows and %d columns. Regular Image is 1024x768",rawImage.GetRows(), rawImage.GetCols());

    }


    error = cam.StopCapture();
    error = cam.Disconnect();

}

void deInterleave(Image& RawImage, Image& Right, Image& Left)
{
 return;
}

void PrintError( Error error )
{
    error.PrintErrorTrace();
}



