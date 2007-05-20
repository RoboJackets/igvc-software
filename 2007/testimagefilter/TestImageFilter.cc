/*
 * TestImageFilter.cc
 * By David Foster
 * 
 * This driver takes image data from an input camera device,
 * filters out the red and green channels, and outputs the new
 * images via a new camera device.
 */

#include <libplayercore/playercore.h>
#include "base/imagebase.h"

// -----------------------------------------------------------------------

class TestImageFilter : public ImageBase {
public:
	TestImageFilter(ConfigFile* cf, int section);
	~TestImageFilter();
	
protected:
	virtual int ProcessFrame();
};

// -----------------------------------------------------------------------
#pragma mark -n

TestImageFilter::TestImageFilter(ConfigFile* cf, int section)
	: ImageBase(
		cf, section,
		false,                          // new commands do NOT override old ones
		PLAYER_MSGQUEUE_DEFAULT_MAXLEN, // incoming message queue is as long as possible
		PLAYER_CAMERA_CODE)             // output filtered images via the "camera" interface
{
	if (this->GetError() != 0) {
		// Error in superclass constructor; abort
		return;
	}	
	
	/* Read options from the config file */
	// <no special options>
}

TestImageFilter::~TestImageFilter() {
	// nothing to free/uninit
}

int TestImageFilter::ProcessFrame() {
	player_camera_data_t& image = this->stored_data;
	
	// Make sure the input is in the format we expect
	if ((image.bpp != 24) ||
	    (image.format != PLAYER_CAMERA_FORMAT_RGB888) ||
	    (image.compression != PLAYER_CAMERA_COMPRESS_RAW))
	{
		PLAYER_ERROR("input image in unexpected format");
		return -1;
	}
	
	// Transform the input image, in place
	int numPixels = image.width * image.height;
	uint8_t *p = image.image;	
	for (int i=0; i<numPixels; i++, p+=3) {
		p[0] = 0; // kill red
		p[1] = 0; // kill green
	}
	
	// Transmit the new filtered image
	size_t size = sizeof(image) - sizeof(image.image) + image.image_count;
	Publish(this->device_addr, NULL,
            PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE,
            reinterpret_cast<void*>(&image), size, NULL);
	
	return 0;
}

// -----------------------------------------------------------------------
#pragma mark -

// Factory creation function. This function is given as an argument when
// the driver is added to the driver table.
Driver* TestImageFilter_Init(ConfigFile* cf, int section) {
	// Create and return a new instance of this driver
	return (Driver*) new TestImageFilter(cf, section);
}

// Registers the driver in the driver table. Called from the
// player_driver_init function that the loader looks for.
int TestImageFilter_Register(DriverTable* table) {
	table->AddDriver("testimagefilter", TestImageFilter_Init);
	return 0;
}

extern "C" int player_driver_init(DriverTable* table) {
	return TestImageFilter_Register(table);
}
