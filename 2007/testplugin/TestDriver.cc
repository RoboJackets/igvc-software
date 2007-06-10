/*
 * TestDriver.cc
 * By David Foster
 * 
 * This driver implements the "testinterface" interface,
 * which is a custom interface.
 * 
 * This code is based on:
 *     player-2.0.3/server/drivers/laser/urglaserdriver.cc
 */

#include <libplayercore/playercore.h>
#include <unistd.h>		// for usleep()

#include "testinterface.h"

// -----------------------------------------------------------------------

class TestDriver : public Driver {
public:
	TestDriver(ConfigFile* cf, int section);
	~TestDriver();
	
	int Setup();
	int Shutdown();
	
	// This method will be invoked on each incoming message
	virtual int ProcessMessage(MessageQueue* resp_queue,
	                           player_msghdr* hdr,
	                           void* data);

private:
	// Main function for device thread.
	virtual void Main();
	
	const char* message;
};

// -----------------------------------------------------------------------
#pragma mark -n

/** Reads options from the configuration file and does any pre-Setup() setup. */
TestDriver::TestDriver(ConfigFile* cf, int section)
	: Driver(
		cf, section,
		false,                          // new commands do NOT override old ones
		PLAYER_MSGQUEUE_DEFAULT_MAXLEN, // incoming message queue is as long as possible
		PLAYER_TESTINTERFACE_CODE)      // interface ID; see <libplayercore/player.h> for standard interfaces
{
	/* Read options from the config file */
	this->message = cf->ReadString(section, "message", "Hello World!");
}

TestDriver::~TestDriver() {
	// nothing to free/uninit
}

/** Set up the device. Return 0 if things go well, and -1 otherwise. */
int TestDriver::Setup() {
	// Start the device thread; spawns a new thread and executes
	// Main(), which contains the main loop for the driver.
	StartThread();
	
	return 0;
}

/** Shutdown the device. */
int TestDriver::Shutdown() {
	// Stop and join the driver thread
	StopThread();
	
	return 0;
}

/** Processes incoming messages. */
int TestDriver::ProcessMessage(MessageQueue* resp_queue,
                               player_msghdr* hdr,
                               void* data)
{
	if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
	                          PLAYER_TESTINTERFACE_REQ_GET_MESSAGE_LENGTH,
	                          this->device_addr))
	{
		int messageLength = strlen(this->message);
		Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype,
		        &messageLength, sizeof(messageLength),  // message content, message length (in bytes)
		        NULL);                                  // timestamp of message; defaults to current time
		return 0;
	}
	
	return -1;
}

/** Main function for the thread that runs the device. */
void TestDriver::Main()
{
	for (;;) {
		// Terminate if this thread has been cancelled
		pthread_testcancel();
		
		// Process any pending messages
		ProcessMessages();
		
		// Send device state to the client
		Publish(device_addr, NULL, PLAYER_MSGTYPE_DATA,
		        PLAYER_TESTINTERFACE_DATA_MESSAGE,              // message subtype
		        &(this->message), strlen(this->message)+1,      // message content, message length
		        NULL);
		
		// Avoid hogging the CPU
		usleep(10);
	}
}

// -----------------------------------------------------------------------
#pragma mark -

// Factory creation function. This function is given as an argument when
// the driver is added to the driver table.
Driver* TestDriver_Init(ConfigFile* cf, int section) {
	// Create and return a new instance of this driver
	return (Driver*) new TestDriver(cf, section);
}

// Registers the driver in the driver table. Called from the
// player_driver_init function that the loader looks for.
int TestDriver_Register(DriverTable* table) {
	table->AddDriver("test", TestDriver_Init);
	return 0;
}

extern "C" int player_driver_init(DriverTable* table) {
	return TestDriver_Register(table);
}
