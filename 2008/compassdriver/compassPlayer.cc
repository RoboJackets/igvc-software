//the parts of the 1-d interface i need

#include <libplayercore/playercore.h>
#include <unistd.h>		// for usleep()



#include "compass.h"
#include "types.h"

class Compass_Player : public Driver {
	public:
		Compass_Player(ConfigFile* cf, int section);
		~Compass_Player();
	
		int Setup();
		int Shutdown();
		
		// This method will be invoked on each incoming message
		virtual int ProcessMessage(MessageQueue* resp_queue,
	                           player_msghdr* hdr,
	                           void* data);
	private:
		// Main function for device thread.
		virtual void Main();

		// The actual compass driver
		CompassDriver* driver;	
		// Data to send to server
		player_position1d_data data;
}

Compass_Player::Compass_Player(ConfigFile* cf, int section)
	: Driver(
		cf, section,
		true,							// new commands DO override old ones
		PLAYER_MSGQUEUE_DEFAULT_MAXLEN,	// incoming message queue is as long as possible
		PLAYER_CAMERA_CODE)			// interface ID; see <libplayercore/player.h> for standard interfaces
{	
	/* Read options from the config file */
	ConfigData configdata = 0;
	configdata.declination.flt  = cf->ReadFloat(section, "declination", DEFAULT_DECLINATION);
	configdata.truenorth = cf->ReadByte(section, "truenorth", DEFAULT_TRUE_NORTH);
	configdata.calsamplefreq = cf->ReadByte(section,"calsamplefreq", DEFAULT_CALSAMPLE_FREQ);
	configdata.samplefreq = cf->ReadByte(section,"samplefreq", DEFAULT_SAMPLE_FREQ);
	configdata.period = cf->ReadByte(section,"period", DEFAULT_PERIOD);
	configdata.bigendian = cf->ReadByte(section, "bigendian", DEFAULT_BIG_ENDIAN);
	configdata.dampingsize= cf->ReadByte(section,"dampingsize", DEFAULT_DAMPING_SIZE);
	
	// Create the real driver
	this->driver = new CompassDriver(configdata);
}

player_position1d_data_state give_playerdata(){

	player_position1d_data_state player1d_data = 0;
	compassData datastruc;

	datastruc = compassinstance.GetData();
	
	if(datastruc.Heading != -1){//compass sets heading to -1 for an error
		player1d_data.pos = datastruc.Heading;
	}
	else{
		player1d_data.pos = -1;
	}
	
}

