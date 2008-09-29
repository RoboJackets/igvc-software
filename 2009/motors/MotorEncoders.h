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
		__attribute__((__packed__)) typedef struct { short dl; short dr; unsigned short dt; /*unsigned short time;*/ } reply_t;
		typedef struct { double heading; double linVel; double rotVel } state_t;
		typedef struct { short dl; short dr; unsigned short dt; /*unsigned short time;*/ } deltas;
		
		/* Constructor */
		MotorEncoders(void);

		reply_t getInfo(void); //TODO: get rid of this

		deltas getDeltas(void);

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

	private:
		int comm_mode;
		int func;
		double heading;
		bool setFunc(int ret);
};

#endif /* MOTOR_ENCODERS_H */

