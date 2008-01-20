/*
 * This driver is based on the standard "imageseq" driver,
 * which was written by Andrew Howard.
 * 
 * This incarnation of the imageseq driver was written by David Foster.
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_imageseq imageseq
 * @brief Image file sequencer

The imageseqbmp driver simulates a camera by reading an image sequence
from the filesystem.  The filenames for the image sequence must be
numbered; e.g.: "image_0000.bmp", "image_0001.bmp", "image_0002.bmp",
etc.

Note that only grayscale images are currently supported.

@par Compile-time dependencies

- nada

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
  - Default: "image_%04d.bmp"
  - A printf-style format string for describing the image filenames; the
    format string must contain at most one integer argument.

- loop (int)
  - Default: 0
  - Specify 1 to send the entire sequence of images over and over again.
  - Specify 0 to stop sending images once every image-file in the sequence has been sent.

@par Example 

@verbatim
driver
(
  name "imageseqbmp"
  provides ["camera:1"]
  rate 10
  pattern "image_%04d.bmp"
)
@endverbatim


@todo Add support for color images.

@author David Foster

*/
/** @} */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdlib.h>       // for atoi(3)
//#include <netinet/in.h>   // for htons(3)
#include <math.h>

#include "bmpread.h"

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

  // Loop?
  private: int loop;	// boolean

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
  table->AddDriver("imageseqbmp", ImageSeq_Init);
}

extern "C" int player_driver_init(DriverTable* table) {
	ImageSeq_Register(table);
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
ImageSeq::ImageSeq(ConfigFile *cf, int section)
  : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_CAMERA_CODE)
{
  // Data rate
  this->rate = cf->ReadFloat(section, "rate", 10);

  // Format string
  this->pattern = cf->ReadString(section, "pattern", "image_%08d.pnm");

  this->loop = cf->ReadInt(section, "loop", 0);

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
	if (this->LoadImage(filename) != 0) {
	  if (this->loop) {
        this->frame = 0;
		continue;
	  } else {
        break;
	  }
	}

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
  Image image;
  
  // Load image; currently forces the image to mono

  if (!ImageLoad((char *) filename, &image)) {
	  // Error loading BMP
	  return -1;
  }

  this->data.width = image.sizeX;
  this->data.height = image.sizeY;
  this->data.bpp = 24;
  this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
  this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;
  this->data.image_count = this->data.width * this->data.height * 3;
  
  // Check image size
  if (this->data.image_count > PLAYER_CAMERA_IMAGE_SIZE)
  {
    PLAYER_ERROR1("image size is too large [%d]", this->data.image_count);
    return -1;
  }

  // Copy the pixels
  memcpy(this->data.image, image.data, this->data.image_count);
  
  ImageFree(&image);
  
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
