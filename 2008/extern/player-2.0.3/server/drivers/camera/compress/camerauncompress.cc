/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: jpeg compression and decompression routines
// Author: Nate Koenig, Andrew Howard
// Date: 31 Aug 2004
// CVS: $Id: camerauncompress.cc,v 1.1.2.2 2006/06/07 16:12:45 gerkey Exp $
//
///////////////////////////////////////////////////////////////////////////

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_camerauncompress camerauncompress
 * @brief Image compression

The camerauncompress driver accepts data from another camera device,
uncompresses it, and makes the raw data available on a new
interface.

@par Compile-time dependencies

- libjpeg

@par Provides

- Uncompressed image data is provided via a @ref interface_camera
  device.

@par Requires

- Image data to be uncompressed is read from a @ref interface_camera
  device.

@par Configuration requests

- none

@par Configuration file options

- save (int)
  - Default: 0
  - If non-zero, uncompressed images are saved to disk (with a .ppm extension?)
      
@par Example 

@verbatim
driver
(
  name "camerauncompress"
  provides ["camera:1"]
  requires ["camera:0"]  # Uncompress data from device camera:0
)
@endverbatim

@author Nate Koenig, Andrew Howard

*/
/** @} */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdlib.h>       // for atoi(3)
#include <netinet/in.h>   // for htons(3)
#include <math.h>

#include <libplayercore/playercore.h>
#include <libplayercore/error.h>
#include <libplayerjpeg/playerjpeg.h>

class CameraUncompress : public Driver
{
  // Constructor
  public: CameraUncompress( ConfigFile* cf, int section);

  // Setup/shutdown routines.
  public: virtual int Setup();
  public: virtual int Shutdown();

  // This method will be invoked on each incoming message
  public: virtual int ProcessMessage(MessageQueue* resp_queue,
                                     player_msghdr * hdr,
                                     void * data);

  // Main function for device thread.
  private: virtual void Main();
  
  //private: int UpdateCamera();
  private: void WriteData();
  private: void SaveFrame(const char *filename);

  // Input camera device
  private:

    // Camera device info
    Device *camera;
    player_devaddr_t camera_id;
    double camera_time;
    bool camera_subscribed;
    bool NewCamData;
	
    // Acquired camera data
    player_camera_data_t camera_data;
    char converted[PLAYER_CAMERA_IMAGE_SIZE];

    // Output (compressed) camera data
    private: player_camera_data_t data;

    // Save image frames?
    private: int save;
    private: int frameno;
};


Driver *CameraUncompress_Init(ConfigFile *cf, int section)
{
  return ((Driver*) (new CameraUncompress(cf, section)));
}

void CameraUncompress_Register(DriverTable *table)
{
  table->AddDriver("camerauncompress", CameraUncompress_Init);
}

CameraUncompress::CameraUncompress( ConfigFile *cf, int section)
  : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_CAMERA_CODE)
{
  this->frameno = 0;

  this->camera = NULL;
  // Must have a camera device
  if (cf->ReadDeviceAddr(&this->camera_id, section, "requires",
                       PLAYER_CAMERA_CODE, -1, NULL) != 0)
  {
    this->SetError(-1);    
    return;
  }
  this->camera_time = 0.0;

  this->save = cf->ReadInt(section,"save",0);

  // camera settings
  this->NewCamData = false;

  return;
}

int CameraUncompress::Setup()
{
  // Subscribe to the laser.
  if(Device::MatchDeviceAddress(this->camera_id, this->device_addr))
  {
    PLAYER_ERROR("attempt to subscribe to self");
    return(-1);
  }
  if(!(this->camera = deviceTable->GetDevice(this->camera_id)))
  {
    PLAYER_ERROR("unable to locate suitable camera device");
    return(-1);
  }
  if(this->camera->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to camera device");
    return(-1);
  }

  // Start the driver thread.
  this->StartThread();

  return 0;
}

int CameraUncompress::Shutdown()
{
  // Stop the driver thread
  StopThread();
  
  camera->Unsubscribe(InQueue);

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int CameraUncompress::ProcessMessage(MessageQueue* resp_queue, player_msghdr * hdr, 
                               void * data)
{
  assert(hdr);
  assert(data);
  
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, camera_id))
  {
    assert(hdr->size >= sizeof(this->camera_data)-sizeof(camera_data.image));
    if (this->camera_data.compression != PLAYER_CAMERA_COMPRESS_JPEG)
      PLAYER_WARN("uncompressing raw camera images (not good)");
    Lock();
    if (this->NewCamData)
    {
    	Unlock();
    	return 0;
    }
    else
    {
        memcpy(&camera_data, data, hdr->size);
    	this->NewCamData=true;
	    Unlock();
    }
    return 0;
  }
 
  return -1;
}

void CameraUncompress::Main()
{
  size_t size;
  char filename[256];

  while (true)
  {
    // Let the camera driver update this thread
    //this->camera->Wait();
    InQueue->Wait();

    // Test if we are suppose to cancel this thread.
    pthread_testcancel();

	ProcessMessages();

    // Get the latest camera data
    Lock();
    if (NewCamData)
    {
    	Unlock();
      jpeg_decompress( (unsigned char*)this->data.image, 
      										PLAYER_CAMERA_IMAGE_SIZE,
                                             this->camera_data.image,
                                             this->camera_data.image_count);

      if (this->save)
      {
        snprintf(filename, sizeof(filename), "click-%04d.ppm",this->frameno++);
        FILE *fp = fopen(filename, "w+");
        fwrite (this->data.image, 1, this->data.image_count, fp);
        fclose(fp);
      }

      size = sizeof(this->data) - sizeof(this->data.image) + this->data.image_count;

      this->data.width = (this->camera_data.width);
      this->data.height = (this->camera_data.height);
      this->data.image_count = data.width*data.height*3;
      this->data.bpp = 24;
      this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
      this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;
      
      Publish(device_addr, NULL, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, (void*) &this->data, size, &this->camera_time);
      Lock();
      NewCamData = false;
    }
    Unlock();
  }
  return;
}

