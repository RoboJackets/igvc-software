/*
 * This is a generic driver for DV cameras over the Firewire (IEEE 1394) protocol.
 * 
 * By David Foster
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>

// FIXME: dv1394 is officially deprecated.  It should be replaced when rawiso becomes available.
#include "dv1394.h"

#include "dv/frame.cc"

// -----------------------------------------------------------------------

// This defines whether to expect NTSC frames [TRUE]
// or PAL frames [FALSE] from the DV camera.
// (Our DV camera appears to be NTSC.)
#define DV_IS_NTSC TRUE

#if DV_IS_NTSC
	#define DV1394_FRAME_SIZE DV1394_NTSC_FRAME_SIZE
#else
	#define DV1394_FRAME_SIZE DV1394_PAL_FRAME_SIZE
#endif

// -----------------------------------------------------------------------

class CameraDV1394
{
public:
	CameraDV1394(const char* camDeviceFilepath);
	~CameraDV1394();
	
	/** Connects to the camera. Returns true if successful. */
	bool Connect();
	/** Disconnects from the camera. */
	void Disconnect();
	/** Returns whether this driver is connected to the camera. */
	bool IsConnected() { return this->valid; };
	
	/**
	 * Grabs a frame from the camera.
	 * 
	 * @return	a frame in RGB888 format, or <tt>null</tt> if an error occurred.
	 */
	unsigned char* GrabFrame();
	
	int GetFrameWidth() { return curFrame.GetWidth(); }
	int GetFrameHeight() { return curFrame.GetHeight(); }

private:
	void Resize(int width, int height);

private:
	// This is the path of the camera device in the filesystem.
	const char* camDeviceFilepath;
	
	bool valid;
	Frame curFrame;
	int camDevice;
	int numFrames;
	unsigned char *map;
	struct dv1394_status status;
	unsigned char *rgb;
};

// -----------------------------------------------------------------------
#pragma mark -

CameraDV1394::CameraDV1394(const char* camDeviceFilepath) 
	: valid(false),
	  curFrame(DV1394_FRAME_SIZE)
{
	this->camDeviceFilepath = camDeviceFilepath;
}

CameraDV1394::~CameraDV1394() {
	// Nothing to uninit
}

bool CameraDV1394::Connect() {
	// (The primary setup code is automatically executed when
	//  the "curFrame" instance variable is initialized
	//  and its constructor is invoked.)
	if (!curFrame.IsValid()) {
		// An error occurred while interfacing with the camera
		fprintf(stderr, "DVCamera: Error while interfacing with the DV camera\n");
		return false;
	}
	
	// Open the firewire camera device, with mode open read write
	camDevice = open(camDeviceFilepath, O_RDWR);
	if (camDevice < 0)
	{
		fprintf(stderr, "DVCamera: Can't open %s\n", camDeviceFilepath);
		return false;
	}
	
	// (don't know why this is the case, but Kino agrees)
	numFrames = DV1394_MAX_FRAMES / 4;
	
	//init params are {API Version, ISO channel, number of frames, PAL/NTSC, packet header, packet, "presentation time" offset}
	// KINO: *always* uses PAL (see ieee1394io.cc)
	struct dv1394_init init = {DV1394_API_VERSION, 63, numFrames, DV1394_NTSC, 0, 0, 0};
	if (ioctl(camDevice, DV1394_INIT, &init))
	{
		fprintf(stderr, "DVCamera: dv1394 init failed\n");
		return false;
	}

	// Request that the DV camera begin sending us frames
	if (ioctl(camDevice, DV1394_START_RECEIVE, NULL))
	{
		fprintf(stderr, "DVCamera: Can't start capture\n");
		return false;
	}
	
	// Map the device's frame buffer into memory
	// KINO: does this *before* requesting that the camera begin sending frames
	map = (unsigned char *) mmap(
		NULL,
		DV1394_FRAME_SIZE * numFrames,
		PROT_READ | PROT_WRITE,
		MAP_SHARED,
		camDevice,
		0);
	if (map == MAP_FAILED)
	{
		fprintf(stderr, "DVCamera: mmap failed. Is the NTSC/PAL setting correct?\n");
		return false;
	}

	valid = true;
	return true;
}

void CameraDV1394::Disconnect() {
	if (map)
		munmap(map, DV1394_FRAME_SIZE * numFrames);
	if (camDevice >= 0)
		close(camDevice);
	// (The primary cleanup code is automatically executed when
	//  the "curFrame" instance variable is disposed
	//  and its destructor is invoked.)
	
	if (rgb)
		delete[] rgb;
}

unsigned char* CameraDV1394::GrabFrame() {
	int err;
	struct pollfd pfd;
	
	// Make sure we're connected to the camera
	if (!IsConnected())
		return NULL;

	// Wait for a frame
	pfd.fd = camDevice;
	pfd.events = POLLIN | POLLERR | POLLHUP;
	while ((err = poll(&pfd, 1, 200)) < 0)
	{
		if (err != EAGAIN)
		{
			fprintf(stderr, "DVCamera: poll failed\n");
			return NULL;
		}
	}

	if (ioctl(camDevice, DV1394_RECEIVE_FRAMES, 1))
	{
		fprintf(stderr, "DVCamera: DV1394_RECEIVE_FRAMES failed\n");
		return NULL;
	}

	if (ioctl(camDevice, DV1394_GET_STATUS, &status))
	{
		fprintf(stderr, "DVCamera: DV1394_GET_STATUS failed\n");
		return NULL;
	}
	if ( status.dropped_frames > 0 )
	{
		printf("DVCamera: dv1394 reported %d dropped frames.\n", status.dropped_frames);
	}
	
	// Find the oldest frame in the camera's ringbuffer
	// XXX: is there a reason we're reading the *oldest* frame
	//      instead of the *newest* frame?
	//int lastFrameID = status.first_clear_frame;
	int lastFrameID = status.active_frame - 1;
	if (lastFrameID < 0)
		lastFrameID = numFrames - 1;
	
	// Calculate the offset to the frame's data
	unsigned char *frameData = map + (lastFrameID * DV1394_FRAME_SIZE);
	
	// Update the Frame object to access the current frame's buffer
	curFrame.data = frameData;
	
	// We got a full frame
	curFrame.bytesInFrame = curFrame.GetFrameSize( );
	
	// Parse the frame's header
	curFrame.ExtractHeader();
	
	if (DV_IS_NTSC != (!curFrame.IsPAL())) {
		printf("DVCamera: WARNING: frame is not in the expected format (NTSC/PAL)\n");
	}
	
	// Update the size of the camera buffer
	Resize(curFrame.GetWidth(), curFrame.GetHeight());
	
	// Decode DV to RGB
	curFrame.ExtractRGB(rgb);

	return rgb;
}

void CameraDV1394::Resize(int width, int height)
{
	static int lastWidth = -1, lastHeight = -1;
	if ((width == lastWidth) && (height == lastHeight))
		return;
	lastWidth = width;
	lastHeight = height;
	
	if (rgb)
		delete[] rgb;
	
	rgb = new unsigned char[width * height * 3];
}
