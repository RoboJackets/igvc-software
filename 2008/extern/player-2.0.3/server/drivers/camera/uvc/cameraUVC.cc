/****************************************************************************\
 *  CameraUVC version 0.1a                                                  *
 *  A Camera Plugin Driver for the Player/Stage robot server                *
 *                                                                          *
 *  Copyright (C) 2006 Raymond Sheh                                         *
 *  rsheh at cse dot unsw dot edu dot au     http://rsheh.cse.unsw.edu.au/  *
 *                                                                          *
 *  A Player/Stage plugin driver for cameras compatible with the Linux      *
 *  UVC camera driver (see http://linux-uvc.berlios.de/), such as the       *
 *  Logitech QuickCam Pro 5000.                                             *
 *                                                                          *
 *                                                                          *
 *  This program is free software; you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation; either version 2 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program; if not, write to the Free Software             *
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston,                   *
 *  MA  02111-1307  USA                                                     *
 *                                                                          *
 *                                                                          *
 *  Portions based on the Player/Stage Sample Plugin Driver                 *
 *  Portions based on luvcview by Laurent Pinchart and Michel Xhaard        *
\****************************************************************************/

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <math.h>

#include <libplayercore/playercore.h>
#include <libplayercore/error.h>

// To prevent the C++ compiler's name mangling from causing 
// problems with the C names here
extern "C"
{
#include "v4l2uvc.h"
}

class CameraUVC : public Driver
{
  public:
    // Player must-have methods
    CameraUVC(ConfigFile* cf, int section);
    int Setup();
    int Shutdown();

    /// @brief Message handler
    public: int ProcessMessage (MessageQueue *resp_queue, player_msghdr *hdr, void *data);

  private: 
    virtual void Main();
    int GrabFrame();            // Grab a frame to videoIn
    void WriteData();            // Write new data to the client (through the server)

    player_camera_data_t data;  // Data to send to client (through the server)
    struct vdIn *videoIn;        // Video capture structure
    char videoDevice[512];      // Video capture device (eg. /dev/video0)
    int format;                  // Video capture format (only V4L2_PIX_FMT_MJPEG tested)
    int grabmethod;              // Video capture grab method (only "1" tested)
    int width;                  // Video frame width (no error checking for invalid values!)
    int height;                  // Video frame height (no error checking for invalid values!) 
};


////////////////////////////////////////////////////////////////////////////////
// Player plugin driver standard functions
////////////////////////////////////////////////////////////////////////////////
Driver* CameraUVC_Init(ConfigFile* cf, int section)
{
  return((Driver*)(new CameraUVC(cf, section)));
}

void CameraUVC_Register(DriverTable* table)
{
  table->AddDriver("camerauvc", CameraUVC_Init);
}

#if 0
extern "C" {
  int player_driver_init(DriverTable* table)
  {
    CameraUVC_Register(table);
    return(0);
  } 
}
#endif

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Also reads config file. 
////////////////////////////////////////////////////////////////////////////////
CameraUVC::CameraUVC(ConfigFile* cf, int section)
    : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_CAMERA_CODE)
{
  format = V4L2_PIX_FMT_MJPEG;
  grabmethod = 1;
  strncpy(this->videoDevice, cf->ReadString(section, "port", "/dev/video0"), sizeof(this->videoDevice));
  this->width = cf->ReadTupleInt(section, "size", 0, 320);
  this->height = cf->ReadTupleInt(section, "size", 1, 240);
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
// Also starts the mainloop. 
////////////////////////////////////////////////////////////////////////////////
int CameraUVC::Setup()
{   
  printf("CameraUVC: Driver initialising\n");

  videoIn = new vdIn;
  if (init_videoIn(videoIn, (char *) videoDevice, width, height, format, grabmethod) < 0)
  {
    PLAYER_ERROR("CameraUVC: Error setting up video capture!");
    this->SetError(-1);
    return(-1); 
  }

  this->StartThread();
  printf("CameraUVC: Driver initialisation done\n");
  return(0);
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device. 
// Also stops the mainloop. 
////////////////////////////////////////////////////////////////////////////////
int CameraUVC::Shutdown()
{
  printf("CameraUVC: Driver shutting down\n");

  StopThread();
  close_v4l2(videoIn);
  delete videoIn;
  videoIn = NULL;

  printf("CameraUVC: Driver shutdown complete\n");
  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Main function (mainloop)
////////////////////////////////////////////////////////////////////////////////
void CameraUVC::Main() 
{
  while (true)
  {
    // Test if we are supposed to cancel this thread.
    pthread_testcancel();

    // Process any pending requests.
    ProcessMessages();

    // Grab the next frame (blocking)
    this->GrabFrame();

    // Write data to server
    this->WriteData();
  }
}


////////////////////////////////////////////////////////////////////////////////
// Process requests.  Returns 1 if the configuration has changed.
// Ignore all requests for now. 
////////////////////////////////////////////////////////////////////////////////
int CameraUVC::ProcessMessage (MessageQueue *resp_queue, player_msghdr *hdr, void *data)
{
  return -1;
}

////////////////////////////////////////////////////////////////////////////////
// Store an image frame into videoIn
////////////////////////////////////////////////////////////////////////////////
int CameraUVC::GrabFrame()
{
  if (videoIn->signalquit)
  {
    if (uvcGrab(videoIn) < 0) 
    {
      PLAYER_ERROR("CameraUVC: Error grabbing frame!");
      return (-1); 
    }
    // Image is now in videoIn->framebuffer in raw form and 
    // videoIn->tmpbuffer in JPEG form. 
  }
  else
  {
    // signalquit? 
    PLAYER_ERROR("CameraUVC: Error, got signalquit?");
    return (-1); 
  }
  return 0;
}



////////////////////////////////////////////////////////////////////////////////
// Write the new frame back to the client (through the server)
////////////////////////////////////////////////////////////////////////////////
void CameraUVC::WriteData()
{
  size_t size;
  // Store image details
  this->data.width = this->width; 
  this->data.height = this->height; 
  this->data.bpp = 24; 
  this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
  this->data.fdiv = 1; 
  this->data.compression = PLAYER_CAMERA_COMPRESS_JPEG;
  this->data.image_count = videoIn->buf.bytesused + DHT_SIZE; 

  // Store image
  memcpy(this->data.image, videoIn->tmpbuffer, this->data.image_count);
  
  // Work out the data size; do this BEFORE byteswapping
  size = sizeof(this->data) - sizeof(this->data.image) + this->data.image_count;

  // Write data to the client (through the server)
  Publish (device_addr, NULL, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, &data, size,NULL);
  return;
}

