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
// Desc: Read image sequence
// Author: Andrew Howard
// Date: 24 Sep 2004
// CVS: $Id: imageseq.cc,v 1.3.2.2 2006/09/23 00:11:34 gerkey Exp $
//
///////////////////////////////////////////////////////////////////////////

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_imageseq imageseq
 * @brief Image file sequencer

The imageseq driver simulates a camera by reading an image sequence
from the filesystem.  The filenames for the image sequence must be
numbered; e.g.: "image_0000.pnm", "image_0001.pnm", "image_0002.pnm",
etc.

Note that only grayscale images are currently supported.

@par Compile-time dependencies

- OpenCV

@par Provides

- This driver supports the @ref interface_camera interface.

@par Requires

- none

@par Configuration requests

- none

@par Configuration file options

- rate (float)
  - Default: 10
  - Data rate (Hz); e.g., rate 20 will generate data at 20Hz.

- pattern (string)
  - Default: "image_%04d.pnm"
  - A printf-style format string for describing the image filenames; the
    format string must contain at most one integer argument.

@par Example 

@verbatim
driver
(
  name "imageseq"
  provides ["camera:1"]
  rate 10
  pattern "image_%04d.pnm"
)
@endverbatim


@todo Add support for color images.

@author Andrew Howard

*/
/** @} */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdlib.h>       // for atoi(3)
#include <netinet/in.h>   // for htons(3)
#include <math.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <libplayercore/playercore.h>


class ImageSeq : public Driver
{
  // Constructor
  public: ImageSeq( ConfigFile *cf, int section);

  // Setup/shutdown routines.
  public: virtual int Setup();
  public: virtual int Shutdown();

  // Main function for device thread.
  private: virtual void Main();

  // Read an image
  private: int LoadImage(const char *filename);
  
  // Write camera data
  private: void WriteData();
    
  // Data rate
  private: double rate;

  // Sequence format string
  private: const char *pattern;

  // Frame number
  private: int frame;

  // Output camera data
  private: player_camera_data_t data;
};


// Initialization function
Driver *ImageSeq_Init(ConfigFile *cf, int section)
{
  return ((Driver*) (new ImageSeq(cf, section)));
}

// Driver registration function
void ImageSeq_Register(DriverTable *table)
{
  table->AddDriver("imageseq", ImageSeq_Init);
}


////////////////////////////////////////////////////////////////////////////////
// Constructor
ImageSeq::ImageSeq(ConfigFile *cf, int section)
  : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_CAMERA_CODE)
{
  // Data rate
  this->rate = cf->ReadFloat(section, "rate", 10);

  // Format string
  this->pattern = cf->ReadString(section, "pattern", "image_%06d.pnm");

  return;
}


////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int ImageSeq::Setup()
{  
  // Start at frame 0
  this->frame = 0;
    
  // Start the driver thread.
  this->StartThread();

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int ImageSeq::Shutdown()
{
  // Stop the driver thread
  StopThread();

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void ImageSeq::Main()
{
  char filename[1024];
  struct timespec req;

  req.tv_sec = (time_t) (1.0 / this->rate);
  req.tv_nsec = (long) (fmod(1e9 / this->rate, 1e9));

  while (1)
  {
    pthread_testcancel();
    if (nanosleep(&req, NULL) == -1)
      continue;    
    
    // Test if we are suppose to cancel this thread.
    pthread_testcancel();

    // Compose filename
    snprintf(filename, sizeof(filename), this->pattern, this->frame);

    // Load the image
    if (this->LoadImage(filename) != 0)
      break;

    // Write new camera data
    this->WriteData();
    this->frame++;          
  }
  return;
}


////////////////////////////////////////////////////////////////////////////////
/// Load an image
int ImageSeq::LoadImage(const char *filename)
{
  int i;
  char *src;
  uint8_t *dst;
  IplImage *image;
  
  // Load image; currently forces the image to mono
  image = cvLoadImage(filename, 0);

  this->data.width = image->width;
  this->data.height = image->height;
  this->data.bpp = 8;
  this->data.format = PLAYER_CAMERA_FORMAT_MONO8;
  this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;
  this->data.image_count = this->data.width * this->data.height;
  
  // Check image size
  if (this->data.image_count > PLAYER_CAMERA_IMAGE_SIZE)
  {
    PLAYER_ERROR1("image size is too large [%d]", this->data.image_count);
    return -1;
  }

  // Copy the pixels
  for (i = 0; i < image->height; i++)
  {
    src = image->imageData + i * image->widthStep;
    dst = this->data.image + i * this->data.width;
    memcpy(dst, src, this->data.width);
  }
  
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Write camera data
void ImageSeq::WriteData()
{
  size_t size;
  
  size = sizeof(this->data) - sizeof(this->data.image) + this->data.image_count;
  Publish(device_addr, NULL, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, &this->data, size, NULL);
      
  return;
}
