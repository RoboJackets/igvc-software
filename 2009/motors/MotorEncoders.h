#ifndef MOTOR_ENCODERS_H
#define MOTOR_ENCODERS_H

#include "ArduinoInterface.h"


//TODO: make this use a namespace
//TODO: make this sample regularly
class MotorEncoders: ArduinoInterface {
	public:
		//TODO: move these to .cc file
		//TODO: change these to char arrays
		typedef struct { /*short leftTick; short righttTick;*/ float heading; /*unsigned short time;*/ } reply_t;
		enum  var_t { HEADING = 0 };

		/* Constructor */
		MotorEncoders(void);

		reply_t getInfo(void); //TODO: get rid of this
		//int getLeftTick(void);
		//int getRightTick(void);

		/* Gets the current global heading in radians */
		double getHeading(void);

		/**/
		bool setHeading(double heading);
};

#endif /* MOTOR_ENCODERS_H */

