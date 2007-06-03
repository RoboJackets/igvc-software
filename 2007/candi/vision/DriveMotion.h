#ifndef DRIVE_MOTION_H
#define DRIVE_MOTION_H

#include "MotorOutput.h"

// These calibration constants affect the way a
//     DriveMotion is converted to a MotorOutput.
#define THRUST_POWER 1.00	// drive at max power when thrust is maximum
#define SWIVEL_POWER 1.50

/**
 * Describes a conceptual motion/movement that the robot can perform.
 * 
 * This motion can be converted into actual motor outputs using
 * the 'toMotorOutput' function.
 */
class DriveMotion {
public:
	int thrust;	// robot: -128 = full reverse, +127 = full forward
	// TODO: consider renaming to 'swivel' (which probably is a more intuitive
	//       name for describing this value's effect on the robot)
	int swivel;	// robot: -128 = full right,   +127 = full left
	
	DriveMotion() {}
	DriveMotion(int thrust, int swivel) {
		this->thrust = thrust;
		this->swivel = swivel;
	}
	
	~DriveMotion() {}
	
	/** Converts this DriveMotion into a MotorOutput that (hopefully) performs that motion. */
	MotorOutput toMotorOutput() {
		return MotorOutput(
			minmax((int) (THRUST_POWER*thrust + SWIVEL_POWER*swivel), -128, 127),
			minmax((int) (THRUST_POWER*thrust - SWIVEL_POWER*swivel), -128, 127));
	}
	
private:
	static int minmax(int value, int min, int max) {
		if (value < min) value = min;
		if (value > max) value = max;
		return value;
	}
};

#endif
