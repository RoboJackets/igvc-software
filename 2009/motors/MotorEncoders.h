#ifndef MOTOR_ENCODERS_H
#define MOTOR_ENCODERS_H

#include "ArduinoInterface.h"
#include "EncoderDefines.h"

//TODO: make this use a namespace
//TODO: make this sample regularly
class MotorEncoders: ArduinoInterface {
	public:
		//TODO: move these to .cc file
		//TODO: change these to char arrays

		//http://sig9.com/articles/gcc-packed-structures
		//__attribute__((__packed__)) typedef struct { short leftTick; short righttTick; float heading; /*unsigned short time;*/ } reply_t;
		typedef struct { short leftTick; short righttTick; float heading; /*unsigned short time;*/ } reply_t;
		typedef struct { short leftTick; short righttTick; unsigned short time; /*unsigned short time;*/ } deltas;
		
		/* Constructor */
		MotorEncoders(void);

		reply_t getInfo(void); //TODO: get rid of this

		//int getDeltaTick(void);
		//double getDeltaTime(void);

		deltas getDeltas(void);

		//int getLeftTick(void);
		//int getRightTick(void);

		/* Gets the current global heading in radians */
		double getHeading(void);

		/* Get linear velocity in m per s */	
		double getLinVel(void);

		/* Get rotational velocity in radians per s */
		double getRotVel(void);

		/**/
		bool setHeading(double heading);
	private:
		int comm_mode;
		int ret_mode;
		double heading;
		bool setSendMode(int mode);
		bool setRetMode(int ret);
};

#endif /* MOTOR_ENCODERS_H */

