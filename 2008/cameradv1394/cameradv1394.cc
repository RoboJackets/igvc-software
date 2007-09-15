/*
 * This code is based on:
 *     player-2.0.3/server/drivers/laser/urglaserdriver.cc
 *     player-2.0.3/server/drivers/camera/1394/camera1394.cc
 */

#include <libplayercore/playercore.h>
#include <unistd.h>		// for usleep()

// XXX: create a separate header file and include it instead
#include "cameradv1394driver.cc"

class CameraDV1394_Player : public Driver {
public:
	CameraDV1394_Player(ConfigFile* cf, int section);
	~CameraDV1394_Player();
	
	int Setup();
	int Shutdown();

private:
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

	// The actual camera driver
	CameraDV1394* driver;	
	// Data to send to server
	player_camera_data_t data;
};

// -----------------------------------------------------------------------
#pragma mark -

/** Reads options from the configuration file and does any pre-Setup() setup. */
CameraDV1394_Player::CameraDV1394_Player(ConfigFile* cf, int section)
	: Driver(
		cf, section,
		true,							// new commands DO override old ones
		PLAYER_MSGQUEUE_DEFAULT_MAXLEN,	// incoming message queue is as long as possible
		PLAYER_CAMERA_CODE)			// interface ID; see <libplayercore/player.h> for standard interfaces
{	
	/* Read options from the config file */
	const char* camDeviceFilepath = cf->ReadString(section, "device", "/dev/dv1394");
	
	// Create the real driver
	this->driver = new CameraDV1394(camDeviceFilepath);
}

CameraDV1394_Player::~CameraDV1394_Player() {
	delete this->driver;
}

/** Set up the device. Return 0 if things go well, and -1 otherwise. */
int CameraDV1394_Player::Setup() {
	// Connect to the camera
	if (!driver->Connect())
		return -1;
	
	// Start the device thread; spawns a new thread and executes
	// Main(), which contains the main loop for the driver.
	StartThread();
	
	return 0;
}

/** Shutdown the device. */
int CameraDV1394_Player::Shutdown() {
	bool wasConnected = driver->IsConnected();	
	
	// Disconnect from the camera
	driver->Disconnect();
	
	if (wasConnected) {
		// Stop and join the driver thread
		StopThread();
	}
	
	return 0;
}

/** Processes incoming messages. */
int CameraDV1394_Player::ProcessMessage(MessageQueue* resp_queue,
                                        player_msghdr* hdr,
                                        void* data)
{
	// We don't currently support any messages
	
	return -1;
}

void CameraDV1394_Player::GrabFrame()
{
	int width = driver->GetFrameWidth();
	int height = driver->GetFrameHeight();
	unsigned char* frameData = driver->GrabFrame();
	
	if (frameData == NULL) return;
	
	// Format the image data so that Player can understand it
	this->data.bpp = 24;
	this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
	this->data.image_count = width * height * 3;
	this->data.width = width;
	this->data.height = height;
	memcpy(this->data.image, frameData, this->data.image_count);
}

void CameraDV1394_Player::SendData()
{	
	// Work out the data size
	size_t size = sizeof(this->data) - sizeof(this->data.image) + this->data.image_count;
	this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;
	Publish(this->device_addr, NULL,
            PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE,
            reinterpret_cast<void*>(&this->data), size, NULL);  
}

/** Main function for the thread that runs the device. */
void CameraDV1394_Player::Main()
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
Driver* CameraDV1394_Player_Init(ConfigFile* cf, int section) {
	// Create and return a new instance of this driver
	return (Driver*) new CameraDV1394_Player(cf, section);
}

// Registers the driver in the driver table. Called from the
// player_driver_init function that the loader looks for.
int CameraDV1394_Player_Register(DriverTable* table) {
	table->AddDriver("cameradv1394", CameraDV1394_Player_Init);
	return 0;
}

extern "C" int player_driver_init(DriverTable* table) {
	return CameraDV1394_Player_Register(table);
}
