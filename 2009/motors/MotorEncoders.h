#ifndef MOTOR_ENCODERS_H
#define MOTOR_ENCODERS_H

#include "../arduino/ArduinoInterface.h"
#include "../arduino/ArduinoCmds.hpp"
//#include "MotorEncodersFrame.h"
#include "./ArduinoMotorEncoders/EncoderDefines.h"
#include <string>

using namespace std;

//typedef struct __attribute__((__packed__)) { unsigned int timestamp; unsigned int packetnum; short dl; short dr; unsigned short dt; } reply_t;

//TODO: make this use a namespace
//TODO: make this sample regularly
class MotorEncoders: public ArduinoInterface {
	public:
		//TODO: move these to .cc file
		//TODO: change these to char arrays

		//typedef struct { double heading; double linVel; double rotVel } state_t;
		//typedef struct { long timestamp; long packetnum; short dl; short dr; unsigned short dt; } reply_t;
		//typedef reply_t EncoderPacket;
		//typedef struct __attribute__((__packed__)) { short command; short len; byte * arg; } command_t;

		/* Constructor */
		MotorEncoders(void);

		/* Gets the current global heading in radians */
		double getHeading(void);

		/* Get linear velocity in m per s */	
		double getLinVel(void);

		/* Get rotational velocity in radians per s */
		double getRotVel(void);

		/* Set global heading */
		void setHeading(double heading);

		/* set to Push or Pull mode*/
		bool setSendMode(int mode);

		/* set to Push or Pull mode, set interogation rate (delay between data in ms)*/
		bool setSendMode(int mode, int int_rts);

		bool setArduinoClock();

		bool setLogFile(string str);

		//bool setLogging(bool status);

		DataPacket::encoder_reply_t getInfo(void);

		bool getInfo_class(DataPacket * status); //TODO: get rid of this



	private:
		int comm_mode;
		int func;
		double heading;
		string logfile;
		//bool logging_on;
		bool setFunc(int ret);

		//unsigned int rx_packetnum;
		//unsigned int tx_packetnum;

		//EncoderPacket getDeltas(void);

};


#endif /* MOTOR_ENCODERS_H */

