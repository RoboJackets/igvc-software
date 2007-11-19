//the parts of the 1-d interface i need


//g++ -I /usr/local/include/player-2.0/ compassPlayer.cc
//#include <player-2.0/libplayercore/playercore.h>
#include <player-2.0/libplayercore/playercore.h>
#include <unistd.h>		// for usleep()
#include <math.h>		//for M_PI


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
		//player_position1d_data playerdata = {0};
		player_position1d_data playerdata;
};

Compass_Player::Compass_Player(ConfigFile* cf, int section): Driver(
		cf, section,
		true,							// new commands DO override old ones
		PLAYER_MSGQUEUE_DEFAULT_MAXLEN,	// incoming message queue is as long as possible
		PLAYER_POSITION1D_CODE)			// interface ID; see <libplayercore/player.h> for standard interfaces
{	
	/* Read options from the config file */
	ConfigData configdata = {0};
	/*configdata.declination.flt  = cf->ReadFloat(section, "declination", DEFAULT_DECLINATION);
	configdata.truenorth = cf->ReadByte(section, "truenorth", DEFAULT_TRUE_NORTH);
	configdata.calsamplefreq = cf->ReadByte(section,"calsamplefreq", DEFAULT_CALSAMPLE_FREQ);
	configdata.samplefreq = cf->ReadByte(section,"samplefreq", DEFAULT_SAMPLE_FREQ);
	configdata.period = cf->ReadByte(section,"period", DEFAULT_PERIOD);
	configdata.bigendian = cf->ReadByte(section, "bigendian", DEFAULT_BIG_ENDIAN);
	configdata.dampingsize= cf->ReadByte(section,"dampingsize", DEFAULT_DAMPING_SIZE);*/

	configdata.declination.flt  = cf->ReadFloat(section, "declination", DEFAULT_DECLINATION);
	configdata.truenorth = (uint_8)cf->ReadInt(section, "truenorth", DEFAULT_TRUE_NORTH);//cast form int to uint_8
	configdata.calsamplefreq = (uint_8)cf->ReadInt(section,"calsamplefreq", DEFAULT_CALSAMPLE_FREQ);
	configdata.samplefreq = (uint_8)cf->ReadInt(section,"samplefreq", DEFAULT_SAMPLE_FREQ);
	configdata.period = (uint_8)cf->ReadInt(section,"period", DEFAULT_PERIOD);
	configdata.bigendian = (uint_8)cf->ReadInt(section, "bigendian", DEFAULT_BIG_ENDIAN);
	configdata.dampingsize= (uint_8)cf->ReadInt(section,"dampingsize", DEFAULT_DAMPING_SIZE);


	DataRespType dataresptype = {0};
	dataresptype.XRaw = cf->ReadBool(section, "XRaw", DEFAULT_XRAW);
	dataresptype.YRaw = cf->ReadBool(section, "YRaw", DEFAULT_YRAW);
	dataresptype.XCal = cf->ReadBool(section, "XCal", DEFAULT_XCAL);
	dataresptype.YCal = cf->ReadBool(section, "YCal", DEFAULT_YCAL);
	dataresptype.Heading = cf->ReadBool(section, "Heading", DEFAULT_HEADING);
	dataresptype.Magnitude = cf->ReadBool(section, "Magnitude", DEFAULT_MAGNITUDE);
	dataresptype.Temperature = cf->ReadBool(section, "Temperature", DEFAULT_TEMPERATURE);
	dataresptype.Distortion = cf->ReadBool(section, "Distortion", DEFAULT_DISTORTION);
	dataresptype.CalStatus = cf->ReadBool(section, "CalStatus", DEFAULT_CALSTATUS);
	
	// Create the real driver
	this->driver = new CompassDriver(configdata, dataresptype);
}

Compass_Player::~Compass_Player(){
	delete this->driver;
}
int Compass_Player::Setup(){
	//the real driver constructor loaded config, or we could set the config and requested data types here instead
	StartThread();
	
	return 0;
}

int Compass_Player::Shutdown(){
	//
	StopThread();
	return 0;
}

int Compass_Player::ProcessMessage(MessageQueue* resp_queue, player_msghdr* hdr, void* data){
	if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION1D_DATA_STATE,
					device_addr) )
	{
		// TODO: implement
		//player_position1d_data playerdata = {0};
		compassData data = {0};
		//playerdata = {0};
		data = driver->GetData();

		if(data.Heading != -1){//-1 indicates error
			this->playerdata.pos = ((data.Heading)*M_PI/180);//player expects rad, compass gives degrees -- is there a better way/is this really what i want to do?.
			size_t size = sizeof(this->playerdata);
			Publish(this->device_addr, NULL, PLAYER_MSGTYPE_DATA, PLAYER_POSITION1D_DATA_STATE, reinterpret_cast<void*>(&this->playerdata), size, NULL);

			return(0);
		}
		else{
			return(-1);
		}
		
	}
	else
	{
		return(-1);
	}
}

void Compass_Player::Main(){
	for (;;) {
		// Terminate if this thread has been cancelled
		pthread_testcancel();
		
		// Process any pending messages
		ProcessMessages();
	}
}

