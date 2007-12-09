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
// CVS: $Id: cameracompress.cc,v 1.12.2.2 2006/09/23 00:11:34 gerkey Exp $
//
///////////////////////////////////////////////////////////////////////////

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_cameracompress cameracompress
 * @brief Image compression

The cameracompress driver accepts data from another camera device,
compresses it, and makes the compressed data available on a new
interface.

@par Compile-time dependencies

- libjpeg

@par Provides

- Compressed image data is provided via a @ref interface_camera
  device.

@par Requires

- Image data to be compressed is read from a @ref interface_camera
  device.

@par Configuration requests

- none

@par Configuration file options

- save (int)
  - Default: 0
  - If non-zero, compressed images are saved to disk (with a .ppm extension?)

- image_quality (float)
  - Default: 0.8
  - JPEG image quality (0.0 - 1.0)
      
@par Example 

@verbatim
driver
(
  name "cameracompress"
  provides ["camera:1"]
  requires ["camera:0"]  # Compress data from device camera:0
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

class CameraCompress : public Driver
{
  // Constructor
  public: CameraCompress( ConfigFile* cf, int section);

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

    // Image quality for JPEG compression
    private: double quality;

    // Save image frames?
    private: int save;
    private: int frameno;
};


Driver *CameraCompress_Init(ConfigFile *cf, int section)
{
  return ((Driver*) (new CameraCompress(cf, section)));
}

void CameraCompress_Register(DriverTable *table)
{
  table->AddDriver("cameracompress", CameraCompress_Init);
}

CameraCompress::CameraCompress( ConfigFile *cf, int section)
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
  this->quality = cf->ReadFloat(section, "image_quality", 0.8);

  // camera settings
  this->NewCamData = false;

  return;
}

int CameraCompress::Setup()
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

  return 0;

  // Start the driver thread.
  this->StartThread();

  return 0;
}

int CameraCompress::Shutdown()
{
  // Stop the driver thread
  StopThread();
  
  camera->Unsubscribe(InQueue);

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int CameraCompress::ProcessMessage(MessageQueue* resp_queue, player_msghdr * hdr, 
                               void * data)
{
  assert(hdr);
  assert(data);
  
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, camera_id))
  {
    assert(hdr->size >= sizeof(this->camera_data)-sizeof(camera_data.image));
    Lock();
    player_camera_data_t * recv = reinterpret_cast<player_camera_data_t * > (data);
    camera_data.width = recv->width;
    camera_data.height = recv->height;
    camera_data.bpp = recv->bpp;
    camera_data.format = recv->format;
    camera_data.fdiv = recv->fdiv;
    camera_data.compression = recv->compression;
    camera_data.image_count = recv->image_count;
    memcpy(camera_data.image, recv->image, recv->image_count);
    this->NewCamData=true;
    Unlock();
    if (this->camera_data.compression != PLAYER_CAMERA_COMPRESS_RAW)
      PLAYER_WARN("compressing already compressed camera images (not good)");
  	
    return 0;
  }
 
  return -1;
}

void CameraCompress::Main()
{
  size_t size;
  char filename[256];
  char * ptr, * ptr1;
  int i, l;

  while (true)
  {
    // Let the camera driver update this thread
    //this->camera->Wait();
    InQueue->Wait();

    // Test if we are suppose to cancel this thread.
    pthread_testcancel();

	ProcessMessages();

    // Get the latest camera data
    if (NewCamData)
    {
      switch (this->camera_data.bpp)
      {
      case 8:
        l = (this->camera_data.width) * (this->camera_data.height);
    	ptr = this->converted; ptr1 = (char *)(this->camera_data.image);
	for (i = 0; i < l; i++)
	{
	  ptr[0] = *ptr1;
	  ptr[1] = *ptr1;
	  ptr[2] = *ptr1;
	  ptr += 3; ptr1++;
	}
	ptr = this->converted;
	break;
      case 24:
        ptr = (char *)(this->camera_data.image);
	break;
      case 32:
        l = (this->camera_data.width) * (this->camera_data.height);
	ptr = this->converted; ptr1 = (char *)(this->camera_data.image);
	for (i = 0; i < l; i++)
	{
          ptr[0] = ptr1[0];
          ptr[1] = ptr1[1];
	  ptr[2] = ptr1[2];
	  ptr += 3; ptr1 += 4;
        }
        ptr = this->converted;
	break;
      default:
        PLAYER_WARN("unsupported image depth (not good)");
        return;
      }
      this->data.image_count = jpeg_compress( (char*)this->data.image, 
                                             ptr,
                                             this->camera_data.width, 
                                             this->camera_data.height,
                                             PLAYER_CAMERA_IMAGE_SIZE, 
                                             (int)(this->quality*100));

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
      this->data.bpp = 24;
      this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
      this->data.compression = PLAYER_CAMERA_COMPRESS_JPEG;
      this->data.image_count = (this->data.image_count);
      
      Publish(device_addr, NULL, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, (void*) &this->data, size, &this->camera_time);
    }
  }
  return;
}
/*
int CameraCompress::UpdateCamera()
{
  size_t size;

  // Get the camera data.
  size = this->camera->GetData(this->camera_id, 
                               &this->camera_data,
                               sizeof(this->camera_data), &this->camera_time);
  
  // Do some byte swapping
  this->camera_data.width = ntohs(this->camera_data.width);
  this->camera_data.height = ntohs(this->camera_data.height); 
  this->camera_data.bpp = this->camera_data.bpp;
  this->camera_data.format = this->camera_data.format;
  this->camera_data.image_size = ntohl(this->camera_data.image_size);

  if (this->camera_data.compression != PLAYER_CAMERA_COMPRESS_RAW)
    PLAYER_WARN("compressing already compressed camera images (not good)");

  return 1;
}*/
