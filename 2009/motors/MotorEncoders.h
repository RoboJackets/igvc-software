#ifndef MOTOR_ENCODERS_H
#define MOTOR_ENCODERS_H

#include "../arduino/ArduinoInterface.h"
#include "./ArduinoMotorEncoders/EncoderDefines.h"
#include <string>

using namespace std;

//TODO: make this use a namespace
//TODO: make this sample regularly
class MotorEncoders: ArduinoInterface {
	public:
		//TODO: move these to .cc file
		//TODO: change these to char arrays

		//typedef struct { double heading; double linVel; double rotVel } state_t;
		//typedef struct { long timestamp; long packetnum; short dl; short dr; unsigned short dt; } EncoderPacket;
		typedef struct __attribute__((__packed__)) { long timestamp; long packetnum; short dl; short dr; unsigned short dt; } reply_t;
		typedef struct __attribute__((__packed__)) { short command; short len; byte * arg; } command_t;

		/* Constructor */
		MotorEncoders(void);

		/* Gets the current global heading in radians */
		double getHeading(void);

		/* Get linear velocity in m per s */	
		double getLinVel(void);

		/* Get rotational velocity in radians per s */
		double getRotVel(void);

		/* Set global heading */
		bool setHeading(double heading);

		/* set to Push or Pull mode*/
		bool setSendMode(int mode);

		/* set to Push or Pull mode, set interogation rate (delay between data in ms)*/
		bool setSendMode(int mode, int int_rts);

		bool setArduinoClock();

		bool setLogFile(string str);

		//bool setLogging(bool status);

	private:
		int comm_mode;
		int func;
		double heading;
		string logfile;
		//bool logging_on;
		bool setFunc(int ret);

		unsigned int rx_packetnum;
		unsigned int tx_packetnum;

		reply_t getInfo(void); //TODO: get rid of this

		//EncoderPacket getDeltas(void);

};

#endif /* MOTOR_ENCODERS_H */

