/*
 * This code is based on:
 *     player-2.0.3/server/drivers/laser/urglaserdriver.cc
 *     player-2.0.3/server/drivers/camera/1394/camera1394.cc
 */

#include <libplayercore/playercore.h>
#include <unistd.h>		// for usleep()
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

typedef struct dv_decoder_s dv_decoder_t;

// -----------------------------------------------------------------------

class CameraDV1394 : public Driver {
public:
	CameraDV1394(ConfigFile* cf, int section);
	~CameraDV1394();
	
	int Setup();
	int Shutdown();

private:
	void Resize(int width, int height);
	void GrabFrame();
	void SendData();

public:
	// This method will be invoked on each incoming message
	virtual int ProcessMessage(MessageQueue* resp_queue,
	                           player_msghdr* hdr,
	                           void* data);

private:
	// Main function for device thread.
	virtual void Main();
	
	// This is the path of the camera device in the filesystem.
	const char* camDeviceFilepath;
	
	bool valid;
	Frame curFrame;
	int camDevice;
	int numFrames;
	unsigned char *map;
	struct dv1394_status status;
	unsigned char *rgb;
	
	// Data to send to server
	player_camera_data_t data;
};

// -----------------------------------------------------------------------
#pragma mark -

/** Reads options from the configuration file and does any pre-Setup() setup. */
CameraDV1394::CameraDV1394(ConfigFile* cf, int section)
	: Driver(
		cf, section,
		true,							// new commands DO override old ones
		PLAYER_MSGQUEUE_DEFAULT_MAXLEN,	// incoming message queue is as long as possible
		PLAYER_CAMERA_CODE),			// interface ID; see <libplayercore/player.h> for standard interfaces
	  valid(false),
	  curFrame(DV1394_FRAME_SIZE)
{
	/* Read options from the config file */
	this->camDeviceFilepath = cf->ReadString(section, "device", "/dev/dv1394");
}

CameraDV1394::~CameraDV1394() {
	// Nothing to uninit
}

/** Set up the device. Return 0 if things go well, and -1 otherwise. */
int CameraDV1394::Setup() {
	// (The primary setup code is automatically executed when
	//  the "curFrame" instance variable is initialized
	//  and its constructor is invoked.)
	if (!curFrame.IsValid()) {
		// An error occurred while interfacing with the camera
		fprintf(stderr, "DVCamera: Error while interfacing with the DV camera\n");
		return -1;
	}
	
	// Open the firewire camera device, with mode open read write
	camDevice = open(camDeviceFilepath, O_RDWR);
	if (camDevice < 0)
	{
		fprintf(stderr, "DVCamera: Can't open %s\n", camDeviceFilepath);
		return -1;
	}
	
	// (don't know why this is the case, but Kino agrees)
	numFrames = DV1394_MAX_FRAMES / 4;
	
	//init params are {API Version, ISO channel, number of frames, PAL/NTSC, packet header, packet, "presentation time" offset}
	// KINO: *always* uses PAL (see ieee1394io.cc)
	struct dv1394_init init = {DV1394_API_VERSION, 63, numFrames, DV1394_NTSC, 0, 0, 0};
	if (ioctl(camDevice, DV1394_INIT, &init))
	{
		fprintf(stderr, "DVCamera: dv1394 init failed\n");
		return -1;
	}

	// Request that the DV camera begin sending us frames
	if (ioctl(camDevice, DV1394_START_RECEIVE, NULL))
	{
		fprintf(stderr, "DVCamera: Can't start capture\n");
		return -1;
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
		return -1;
	}

	valid = true;
	
	// Start the device thread; spawns a new thread and executes
	// Main(), which contains the main loop for the driver.
	StartThread();
	
	return 0;
}

/** Shutdown the device. */
int CameraDV1394::Shutdown() {
	if (map)
		munmap(map, DV1394_FRAME_SIZE * numFrames);
	if (camDevice >= 0)
		close(camDevice);
	// (The primary cleanup code is automatically executed when
	//  the "curFrame" instance variable is disposed
	//  and its destructor is invoked.)
	
	if (rgb)
		delete[] rgb;
	
	if (this->valid) {
		// Stop and join the driver thread
		StopThread();
	}
	
	return 0;
}

/** Processes incoming messages. */
int CameraDV1394::ProcessMessage(MessageQueue* resp_queue,
                               player_msghdr* hdr,
                               void* data)
{
	// We don't currently support any messages
	
	return -1;
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

void CameraDV1394::GrabFrame()
{
	int err;
	struct pollfd pfd;

	// Wait for a frame
	pfd.fd = camDevice;
	pfd.events = POLLIN | POLLERR | POLLHUP;
	while ((err = poll(&pfd, 1, 200)) < 0)
	{
		if (err != EAGAIN)
		{
			fprintf(stderr, "DVCamera: poll failed\n");
			return;
		}
	}

	if (ioctl(camDevice, DV1394_RECEIVE_FRAMES, 1))
	{
		fprintf(stderr, "DVCamera: DV1394_RECEIVE_FRAMES failed\n");
		return;
	}

	if (ioctl(camDevice, DV1394_GET_STATUS, &status))
	{
		fprintf(stderr, "DVCamera: DV1394_GET_STATUS failed\n");
		return;
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
	
	// Format the image data so that Player can understand it
	this->data.bpp = 24;
	this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
	this->data.image_count = curFrame.GetWidth() * curFrame.GetHeight() * 3;
	this->data.width = curFrame.GetWidth();
	this->data.height = curFrame.GetHeight();
	memcpy(this->data.image, rgb, this->data.image_count);
}

void CameraDV1394::SendData()
{	
	// Work out the data size
	size_t size = sizeof(this->data) - sizeof(this->data.image) + this->data.image_count;
	this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;
	Publish(this->device_addr, NULL,
            PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE,
            reinterpret_cast<void*>(&this->data), size, NULL);  
}

/** Main function for the thread that runs the device. */
void CameraDV1394::Main()
{
	for (;;) {
		// Terminate if this thread has been cancelled
		pthread_testcancel();
		
		// Process any pending messages
		ProcessMessages();
		
		GrabFrame();
		SendData();

		/*
		// Avoid hogging the CPU
		usleep(10);
		*/
	}
}

// -----------------------------------------------------------------------
#pragma mark -

// Factory creation function. This function is given as an argument when
// the driver is added to the driver table.
Driver* CameraDV1394_Init(ConfigFile* cf, int section) {
	// Create and return a new instance of this driver
	return (Driver*) new CameraDV1394(cf, section);
}

// Registers the driver in the driver table. Called from the
// player_driver_init function that the loader looks for.
int CameraDV1394_Register(DriverTable* table) {
	table->AddDriver("cameradv1394", CameraDV1394_Init);
	return 0;
}

extern "C" int player_driver_init(DriverTable* table) {
	return CameraDV1394_Register(table);
}
