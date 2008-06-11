#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H

class MotorOutput {
public:
	int leftSpeed;  // left wheel:  -128 = full reverse, +127 = full forward
	int rightSpeed; // right wheel: -128 = full reverse, +127 = full forward
	
	MotorOutput() {}
	MotorOutput(int leftSpeed, int rightSpeed) {
		this->leftSpeed = leftSpeed;
		this->rightSpeed = rightSpeed;
	}
	
	~MotorOutput() {}
};

#endif
